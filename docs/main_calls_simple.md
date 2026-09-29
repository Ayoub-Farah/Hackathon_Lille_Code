# Appels principaux du programme

Le [schéma Graphviz simplifié](main_calls_simple.dot) présente les principaux
appels du [code actuel](../src/main.cpp), sans détailler les calculs ni les
branches de validation.

- `main()` appelle `setup_routine()`, qui configure le matériel et active les tâches.
- Toutes les **200 µs**, `loop_critical_task()` lit les mesures, vérifie les limites,
  décide l'insertion locale et commande la puissance. SM1 prépare ensuite les
  nombres d'insertion du prochain tour et démarre les échanges.
- À chaque réception RS485, `reception_function()` traite la trame et déclenche
  l'envoi local après le prédécesseur : **SM1 → SM2 → … → SM10**.
- `loop_communication_task()` lit la console ; `loop_background_task()` gère
  les LED et l'export des acquisitions sur SM1.

Les flèches pleines montrent les appels ; les pointillées représentent
l'activation des tâches et l'enregistrement du callback. Ces tâches ne sont
pas des appels successifs de `main()`. L'identification de la carte à
l'initialisation globale et les helpers secondaires sont omis.

Pour générer le SVG depuis la racine du dépôt avec Graphviz :

```sh
dot -Tsvg docs/main_calls_simple.dot -o docs/main_calls_simple.svg
```
