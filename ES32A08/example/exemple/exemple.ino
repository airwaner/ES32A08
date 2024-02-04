#include "ES32A08.h"

ES32A08 board;  // Déclaration de la carte ES32A08 en tant que "board".

bool etatPWRLed = 1;
bool decimalPointOn = false; // Utilisé pour l'exemple du point décimal clignotant 1 fois par seconde.
bool fiveOn = false; // Pour suivre l'état du "5"

unsigned long previousUpdateTime = 0; // Stocke la dernière fois que l'affichage a été mis à jour
const long refreshInterval = 40; // Intervalle de rafraichissement inter-digits en millisecondes

void setup() {
  Serial.begin(115200);                 // initialiser le port série.
  board.begin();                        // Initialise la carte et toutes ses possibilités.
  board.togglePWRLEDAsHeartbeat(true);  // Fait clignoter la led PWR si le booléen bool heartbeatEnabled = true; Sinon, LedPWR reste commandable par la fonction board.setPWRLED(1 ou 0).
  //board.setPWRLED(0); // Forcer l'extinction de la Led PWR.
  delay(1000);  // Attends 1 seconde

  board.delay1.start(1000); // Pour le point décimal, clignote chaque seconde
  board.delay2.start(5000); // Pour le "5", clignote toutes les 2 secondes
  board.delay3.start(10000);  // Démarrer le troisième délai de 10 secondes, si nécessaire
}


void loop() {
// delay(1000); // temps programme 50Hz suffisant en réactivité pour notre exemple.

// Faire clignoter 1 fois par seconde le dernier point décimal en utilisant les délais non-bloquants de la bibliothèque.
// Combine également l'affichage d'un "5" en prenant en compte l'affichage du Point décimal si déjà affiché.
//toggleDecimalPoint(); // Appel de la fonction pour gérer le clignotement du point décimal
//toggleFive(); //Appel de la fonction pour gérer le clignotement du 5.
//----------------------------------------------------------------------------------------------------------------------

board.afficher("9876"); // Envoi de la chaine de caractères à convertir pour l'afficheur 4 Digits.
unsigned long currentMillis = millis();

if (currentMillis - previousUpdateTime >= refreshInterval) {
        previousUpdateTime = currentMillis; // Mise à jour du temps pour le dernier rafraîchissement
        board.refreshDisplay(); // Rafraîchit l'affichage seulement après l'interval spécifié
    }
  board.updatePWRLED();  // Fais clignoter à chaque itération du programme si la fonction togglePWRLEDAsHeartbeat(true); à étét déclarée en setup().
 
   if (board.delay2.isCompleted()) {
    Serial.println("Délai 2 terminé");
    
  // Test des relais 

    for (byte relay = 1; relay <= 8; relay++) {
    byte relaysState = 1 << (relay - 1);  // Calcule l'état des relais pour activer relay
    board.updateRelays(relaysState);
    delay(50);
  }
  for (byte relay = 8; relay >= 1; relay--) {
    byte relaysState = 1 << (relay - 1);  // Calcule l'état des relais pour activer relay
    board.updateRelays(relaysState);
    delay(50);
  }

   board.updateRelays(0b00000000);  // Désactive tous les relais

  }

  if (board.delay3.isCompleted()) {
    Serial.println("Délai 3 terminé");
    board.delay3.start(10000);  // Optionnel: redémarrer le délai, si vous utilisez delay3
  }

// Affiche le numéro du bouton appuyé (Key1-Key4) et l'affiche à l'emplacement du N° du Bouton.
// Allume Également le temps de l'appui bouton le relais associé à son numéro.

  for (int i = 1; i <= 4; i++) {
    while (board.readButton(i)) {
      board.updateDisplay(board.digitNumber[i], board.digitToSegment[i]);
      board.setRelay(i, true);  // Active le relais concerné par le bouton.
    }
    board.setRelay(i, false);  // Active le relais concerné par le bouton.
  }

/*
  // Test des chiffres sur chaque digit
  for (byte digit = 0x01; digit <= 0x08; digit <<= 1) {  // Changez selon la logique de vos digits
    for (byte num = 0; num < 11; num++) {
      board.updateDisplay(~digit, board.digitToSegment[num]);  // ~digit si 0 active le digit
    }
  }
*/

  // Pilotage d'un seul relais en particulier
  // La fonction .setRelay(numéro de relais, état) reègle l'état d'un relais en particulier en prenant en compte l'état actuel des autres relais.
  // L'état des autres relais reste alors inchangé.
  board.setRelay(8, true);  // Active le relais 1

  board.setRelay(7, true);  // Désactive le relais 1

 // board.setRelay(1, false);  // Active le relais 2

 // board.setRelay(2, false);  // Désactive le relais 2

  // Lecture et affichage des entrées courant (4-20mA) et tension (0-10V).
  for (int i = 0; i < 4; i++) {
    float current = board.readAnalogmA(i);       // Remplacez i par la broche ADC correspondante
    float voltage = board.readAnalogVoltage(i);  // Idem
    Serial.print("Entrée ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(current);
    Serial.print(" mA, ");
    Serial.print(voltage);
    Serial.println(" V");
  }

  // Lecture de l'entrée numérique n°8 en particulier.
  bool entreeNumeriqueN8 = board.getDigitalInputState(8);
  Serial.print("Entrée Numérique N°8 ");
  Serial.print(8);
  Serial.print(": ");
  Serial.println(entreeNumeriqueN8 ? "HIGH" : "LOW");

  // Lecture de l'état des 8 entrées, et affichage en binaire 8 bits pour visualisation directe.
  String digitalInputsBinary = board.getFormattedDigitalInputs();
  Serial.print("Binaire reflétant les entrées numériques : ");
  Serial.println(digitalInputsBinary);  // Affiche l'état des entrées numériques en format binaire de 8 bits

  // Lecture et affichage des entrées Boutons (Key 1 --> Key 4)
  for (int i = 1; i <= 4; i++) {
    if (board.readButton(i)) {
      Serial.print("Bouton ");
      Serial.print(i);
      Serial.println(" pressé");
    }
  }
}