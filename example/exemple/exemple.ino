#include "ES32A08.h"

ES32A08 board;  // Déclaration de la carte ES32A08 en tant que "board".

bool etatPWRLed = 1;

void setup() {
  Serial.begin(115200);                 // initialiser le port série.
  board.begin();                        // Initialise la carte et toutes ses possibilités.
  board.togglePWRLEDAsHeartbeat(true);  // Fait clignoter la led PWR si le booléen heartbeatEnabled = true; Sinon, LedPWR reste commandable par la fonction board.setPWRLED(1 ou 0).
  //board.setPWRLED(0); // Forcer l'extinction de la Led PWR.
  delay(1000);  // Attends 1 seconde

  board.delay1.start(1000); // Commencez un délai non bloquant de 500 ms
//  board.delay2.start(5000); // Pour le "5", clignote toutes les 2 secondes
//  board.delay3.start(10000);  // Démarrer le troisième délai de 10 secondes, si nécessaire

    // ou
//    board.beginDisplayValue("A123", 500); // Exemple avec une chaîne de caractères avec un temps
}


void loop() {

if (board.delay1.isRunning()) {
board.afficher("Menu"); // Exemple avec un nombre
} // Si le délai 1 (non bloquant) est toujours en cours

if (board.delay1.isCompleted()) {
board.delay2.start(1000); 
board.afficher(" 0. "); // Exemple avec un nombre
} // Si le délai 1 (non bloquant) est terminé (valide 1 seule itération)

if (board.delay2.isRunning()) {
board.afficher("Test"); // Exemple avec un nombre
} // Si le délai 1 (non bloquant) est toujours en cours

  board.updatePWRLED();  // Fais clignoter à chaque itération du programme si la fonction togglePWRLEDAsHeartbeat(true); à étét déclarée en setup().
 
   if (board.delay2.isCompleted()) {
   // Serial.println("Délai 2 terminé");
    
  // Test des relais 

    for (byte relay = 1; relay <= 8; relay++) {
    byte relaysState = 1 << (relay - 1);  // Calcule l'état des relais pour activer relay
    board.setRelays(relaysState);
    delay(50);
  }
  for (byte relay = 8; relay >= 1; relay--) {
    byte relaysState = 1 << (relay - 1);  // Calcule l'état des relais pour activer relay
    board.setRelays(relaysState);
    delay(50);
  }

   board.setRelays(0b00000000);  // Désactive tous les relais
    
    
    board.delay3.start(10000);  // On démarre maintenant le délai 3 puisque le 2 est terminé
  }

  if (board.delay3.isRunning()) {
    float voltage = board.readAnalogVoltage(0); // Lit la tension sur l'entrée 0-10V N°1
  // Affiche la valeur convertie sur l'afficheur 4 digits
  board.afficher(voltage); // On se sert de l'afficheur pour afficher la valeur. Maxi 4 digits +1 virgule.
  }

// Affiche le numéro du bouton appuyé (Key1-Key4) et l'affiche à l'emplacement du N° du Bouton.
// Allume Également le temps de l'appui bouton le relais associé à son numéro.
  for (int i = 1; i <= 4; i++) {
    while (board.readButton(i)) {
      board.updateDisplay(board.digitNumber[i], board.digitToSegment[i]);
      board.setRelay(i, true);  // Active le relais concerné par le bouton tant que l'appui est conservé
    }
    board.setRelay(i, false);  // Désactive le relais concerné par le relâchement du bouton.
    board.afficher(""); // On efface l'écran car le bouton est relaché!
  }

  // Cas du pilotage d'un seul relais en particulier
  // La fonction .setRelay(numéro de relais, état) reègle l'état d'un relais en particulier en prenant en compte l'état actuel des autres relais.
  // L'état des autres relais précedemment changés reste alors inchangé.
  board.setRelay(8, true);  // Active le relais 8
  board.setRelay(7, true);  // Active le relais 7

  // Lecture et affichage des entrées courant (4-20mA) et tension (0-10V).
  for (int i = 0; i < 4; i++) {
    float current = board.readAnalogmA(i);       // Remplacez i par la broche ADC correspondante
    float voltage = board.readAnalogVoltage(i);  // Idem
    /*
    Serial.print("Entrée ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(current);
    Serial.print(" mA, ");
    Serial.print(voltage);
    Serial.println(" V");
    */
  }

  // Lecture de l'entrée numérique n°8 en particulier.
  bool entreeNumeriqueN8 = board.getDigitalInputState(8);
  /*
  Serial.print("Entrée Numérique N°8 ");
  Serial.print(8);
  Serial.print(": ");
  Serial.println(entreeNumeriqueN8 ? "HIGH" : "LOW");
  */

  // Lecture de l'état des 8 entrées numériques, et affichage en binaire 8 bits pour visualisation directe des 8 entrées.
  String digitalInputsBinary = board.getFormattedDigitalInputs();
  /*
  Serial.print("Binaire reflétant les entrées numériques : ");
  Serial.println(digitalInputsBinary);  // Affiche l'état des entrées numériques en format binaire de 8 bits
  */
  // Lecture et affichage des entrées Boutons (Key 1 --> Key 4)
  for (int i = 1; i <= 4; i++) {
    if (board.readButton(i)) {
      /*
      Serial.print("Bouton ");
      Serial.print(i);
      Serial.println(" pressé");
      */
    }
  }
}