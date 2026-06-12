# Fiche Récapitulative de Câblage (S7-1200 vers Système SCARA)

## Prérequis de Travail

* **Couper le jus :** Les disjoncteurs en haut de l'armoire doivent être baissés.
* **Propreté :** Le cuivre dénudé des fils doit rentrer entièrement dans la borne. Aucun métal nu ne doit dépasser pour éviter les courts-circuits.

---

## 1. Le Circuit de Retour (La Masse / 0V)

C'est le fil obligatoire qui permet au courant de l'automate de fermer sa boucle après avoir traversé le driver.

* **Départ :** Borne **`1M`** (ou `M` côté sorties) située en haut de l'automate Siemens.
* **Arrivée :** Bornes **`PUL-`** et **`DIR-`** sur les drivers L1, L2 et L3.
* **La méthode :** Tire un fil du `M` de l'automate vers le `PUL-` du driver. Sur le driver lui-même, coupe un tout petit bout de fil pour faire un "pont" entre `PUL-` et `DIR-`.

---

## 2. Le Cerveau : Signaux de Mouvement (Sorties PTO)

Ce sont les signaux de commande en 24V envoyés par l'automate. Le PULSE dicte la vitesse, la DIRECTION dicte le sens.

### Axe 1 : L'Épaule (Driver DC.L1 - Gros boîtier bleu à droite)

* Sortie Automate **`%Q0.0`** ➔ Borne Driver **`PUL+`**
* Sortie Automate **`%Q0.1`** ➔ Borne Driver **`DIR+`**

### Axe 2 : La Verticale Z (Driver DC.L2 - Gros boîtier bleu au milieu)

* Sortie Automate **`%Q0.2`** ➔ Borne Driver **`PUL+`**
* Sortie Automate **`%Q0.3`** ➔ Borne Driver **`DIR+`**

### Axe 3 : Le Coude (Driver DC.L3 - Gros boîtier bleu à gauche)

* Sortie Automate **`%Q0.4`** ➔ Borne Driver **`PUL+`**
* Sortie Automate **`%Q0.5`** ➔ Borne Driver **`DIR+`**

---

## 3. L'Actionneur Auxiliaire

### Le Moteur du Tapis Roulant (Conveyor)

* Sortie Automate **`%Q0.6`** ➔ À relier sur l'entrée de commande du relais ou du contacteur qui pilote le moteur du tapis.

---

## 4. Ce que tu ne branches pas

* **L'énergie brute (`+VDC` et `GND`) :** La fameuse "guirlande" est déjà câblée en série sur tous tes drivers.
* **Le Driver DC.L4 (Boîtier noir moyen) :** C'est le 4ème axe (rotation de la pince). Volontairement laissé vide pour cette phase du projet.
* **Le symbole Terre (Traits horizontaux) :** Strictement réservé à la protection du métal du coffret, interdiction d'y brancher un signal de l'automate.