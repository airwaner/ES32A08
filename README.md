#Bibliothèque ES32A08 pour ESP32

La bibliothèque ES32A08 est conçue pour faciliter l'utilisation de la carte d'extension ES32A08 avec l'ESP32. Elle fournit une interface simple pour contrôler les relais, lire des entrées numériques et analogiques, et gérer un affichage à 4 digits et des LEDs.
Caractéristiques

    Contrôle de 8 relais via les registres à décalage 74HC595D.
    Lecture de 8 entrées numériques avec le registre à décalage 74HC165.
    Affichage de valeurs sur un afficheur 4 digits.
    Lecture des entrées analogiques 4-20mA et 0-10V.
    Contrôle d'une LED de puissance et d'une LED intégrée comme indicateur de fonctionnement (heartbeat).
    Délais non bloquants pour des opérations asynchrones.

Installation

Pour utiliser la bibliothèque ES32A08, téléchargez la dernière version depuis cette page GitHub et incluez-la dans votre projet Arduino IDE.

1. Allez dans Sketch > Include Library > Add .ZIP Library dans l'Arduino IDE et sélectionnez le fichier téléchargé.
2. Ou, décompressez le contenu et placez-le dans le répertoire 'libraries' de votre dossier Arduino.



Utilisation
Incluez la bibliothèque dans votre sketch Arduino :

#include <ES32A08.h>
ES32A08 board;



Initialisez la carte dans la fonction setup() :

void setup() {
  board.begin(); // Initialise la carte ES32A08
}

Dans la boucle loop(), vous pouvez utiliser les méthodes fournies par la bibliothèque pour contrôler la carte :

void loop() {
  // Utilisez les méthodes de la bibliothèque ici
}



Exemples
Consultez le dossier examples pour des exemples montrant comment utiliser la bibliothèque pour réaliser différentes tâches.




Contribution
Les contributions à ce projet sont bienvenues ! Si vous souhaitez contribuer, veuillez forker le dépôt, apporter vos changements, et soumettre une pull request.
