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
les mesures locales ; un échange incomplet, une consigne hors de [0, 5], un
cycle incohérent ou des consignes recopiées différentes déclenchent le défaut
de communication. La synchronisation et l'hypothèse d'échanges terminés avant
la tâche critique restent celles du firmware précédent.

Les cinq canaux ScopeMimicry de SM1 sont `vc_self`, `N_upper`, `N_lower`,
`i_upper_ref` et `inserted`.

Les deux premiers octets de la trame sont maintenant les nombres d'insertion,
à la place de la référence sinusoïdale signée. Tous les modules doivent utiliser
ce nouveau firmware ensemble.

## Vérification

`python tests/test_distributed_insertion.py` compile et exécute les fonctions
C++ de décision et de réception extraites du firmware avec `clang++`
(`--compiler` permet de choisir un autre compilateur compatible).
Les tests comparent 52 488 sélections à un tri indépendant : égalités,
deux bras, signes opposés et toutes les consignes de 0 à 5. Ils vérifient
aussi la génération NLM, les sources de courant et les trames incohérentes.

Compilation cible : `pio run -e USB`. La validation du timing et des
commutations sur le convertisseur nécessite un essai matériel.
