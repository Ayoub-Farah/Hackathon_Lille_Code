# Insertion distribuée et équilibrage des condensateurs

SM1 calcule les deux consignes NLM à 50 Hz :

- `N_upper = round(5 * (a + m * sin(angle)) / 2)` ;
- `N_lower = round(5 * (a - m * sin(angle)) / 2)`.

Les deux arrondis sont indépendants, comme dans `phase_geeps_test_nodelay`.
SM1 transmet ces nombres, l'identifiant de cycle, son état et ses mesures.
Chaque module transmet ensuite ses mesures dans l'ordre SM1 → … → SM10,
en recopiant les consignes du cycle.

Au début de la tâche critique suivante (période de 200 µs), chaque module
vérifie que l'échange est complet avant de calculer son rang dans son bras :

- bras supérieur : SM1 à SM5, courant de référence mesuré et transmis par SM1 ;
- bras inférieur : SM6 à SM10, courant de référence mesuré et transmis par SM6 ;
- courant partagé positif : priorité aux tensions les plus faibles ;
- courant partagé négatif : priorité aux tensions les plus fortes ;
- tensions égales : priorité à l'identifiant de module le plus petit.

Un module s'insère si son rang, compté à partir de zéro, est inférieur à la
consigne de son bras. Le classement est recalculé à chaque cycle complet,
même si le nombre demandé n'a pas changé. Il n'y a pas de leader chargé
d'envoyer les commandes individuelles d'insertion.

Le classement utilise les tensions brutes sur 12 bits. Chaque module conserve
aussi sa propre valeur encodée au moment de sa transmission. Le courant utilisé
est le même échantillon décodé chez tous les modules du bras, y compris chez
SM1 et SM6 ; une nouvelle mesure locale ne remplace pas cette référence pour
la décision en cours. Le signe est celui du courant décodé, avec la résolution
du transport existant, sans hystérésis autour de zéro.

La commande de LEG2 vaut 1 pour l'insertion et 0 pour le bypass. Le déphasage
des porteuses est ramené à zéro. Les protections locales continuent d'utiliser
les mesures locales : surtension au-delà de 80 V et surintensité au-delà de
8 A en valeur absolue. Seuls ces défauts de puissance et les erreurs de cycle
(échange incomplet, identifiant incohérent ou non consécutif) provoquent un arrêt.
Les contrôles des consignes NLM et de leur recopie sont supprimés ; les paramètres
de modulation doivent produire des consignes comprises entre 0 et 5.
La synchronisation et l'hypothèse d'échanges terminés avant
la tâche critique restent celles du firmware précédent.

Les commandes console affectent directement `mode` : `p` sur SM1 active
`POWERMODE`, `i` active `IDLEMODE`. Les autres modules suivent l'état reçu de
SM1. Aucun état de requête intermédiaire n'est utilisé. L'application des PWM
attend toujours un échange POWER complet pour disposer des mesures du classement.
Un défaut force `IDLEMODE` ; les cycles IDLE effacent les défauts précédents,
et les limites de puissance sont à nouveau vérifiées. Le redémarrage se fait
avec une nouvelle commande `p` sur SM1 après disparition du défaut.

ScopeMimicry enregistre sur SM1 24 canaux de diagnostic dans un buffer
circulaire de 512 points, au pas de 200 µs. Une fois plein, il remplace les
points les plus anciens au lieu d'arrêter l'acquisition. Un défaut fige le
buffer **après enregistrement du cycle fautif** : on conserve jusqu'à 511
cycles antérieurs (102,2 ms) et le cycle de détection. Il n'y a pas de capture
après le déclenchement ; la découverte du bus ne modifie pas la trace figée.

`p` depuis IDLE démarre une nouvelle capture ; `a` ou `s` la réarme également
et efface l'historique précédent. `i` ou `r` fige manuellement la capture.
Après un défaut, utiliser `r` en IDLE pour exporter **avant de relancer `p`**.
L'export remet les points dans l'ordre chronologique avec l'indice `-1`,
sans changer le format des canaux. Un défaut précoce produit moins de 512 points.

Pour analyser une erreur de communication, la dernière ligne contient le défaut
local/reçu dans `fault`, les modules reçus dans `rx_mask` (bit 0 = SM1, etc.)
et leur nombre dans `rx_count`. `fault_mask` indique les modules ayant transmis
un statut de défaut, pas nécessairement son origine. Les actions `*_prev`
concernent le cycle précédent : les comparer aux consignes et mesures de la
ligne précédente, seulement si le bit correspondant de `action_valid_mask`
est présent. La première ligne n'a pas son cycle précédent dans la capture.

`py tests/test_scope_capture.py` vérifie un défaut après plusieurs tours du
buffer, l'export chronologique, le gel pendant la récupération, le réarmement,
l'arrêt manuel et la capture forcée du défaut entre deux acquisitions espacées.

Les deux premiers octets de la trame sont maintenant les nombres d'insertion,
à la place de la référence sinusoïdale signée. Tous les modules doivent utiliser
ce nouveau firmware ensemble.

## Découverte du bus au démarrage

Après 5 000 cycles de contrôle (1 s), SM1 interroge séparément SM2 à SM10,
à raison d'une requête par fenêtre de 200 µs. La trame conserve ses 12 octets :
le statut `DISCOVERY = 2` indique une requête de présence et
`upper_insert_count` désigne alors le module interrogé, pas une consigne
d'insertion. Seul ce module répond directement à SM1 ; un module sans réponse
ne bloque donc pas les suivants. Tous les modules doivent recevoir ce firmware.

Les PWM restent arrêtées pendant la découverte. SM1 exige les neuf réponses
avec les identifiants de cycle attendus dans un même tour de découverte.
Il recommence les tours incomplets et affiche, dans la tâche de fond, le masque
du dernier tour et les modules sans réponse. SM1 lui-même correspond au bit 0 ;
le masque complet vaut `0x3ff`. La commande `p` est refusée jusqu'au succès.
Après succès, les échanges décentralisés reprennent en IDLE ; `p` active la
puissance avec les contrôles habituels. Un échange normal incomplet arrête
la puissance et relance la découverte, sans redémarrage automatique en puissance.

Ce test vérifie la communication aller-retour dans les fenêtres de contrôle ;
il ne mesure pas directement l'alignement des horloges ni les signaux PWM.
Les tests hôte de ce démarrage se lancent avec
`python tests/test_bus_discovery.py` (compilateur `clang++` requis).

## Vérification

`python tests/test_distributed_insertion.py` compile et exécute les fonctions
C++ de décision et de réception extraites du firmware avec `clang++`
(`--compiler` permet de choisir un autre compilateur compatible).
Les tests comparent 52 488 sélections à un tri indépendant : égalités,
deux bras, signes opposés et toutes les consignes de 0 à 5. Ils vérifient
aussi la génération NLM, les sources de courant, les erreurs de cycle,
les défauts de puissance et les transitions console/PWM.

Compilation cible : `pio run -e USB`. La validation du timing et des
commutations sur le convertisseur nécessite un essai matériel.
