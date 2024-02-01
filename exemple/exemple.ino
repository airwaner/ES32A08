#include "ES32A08.h"

ES32A08 board;

void setup() {
  Serial.begin(115200); // initialiser le port série.
  board.begin(); // Initialise la carte
}

void loop() {

    board.setPWRLED(true); // Allume la LED "PWR"
    delay(100); // Attends 1 seconde
    board.setPWRLED(false); // Éteint la LED "PWR"

/*
// Boucler les 8 relais pour tester le fonctionnement.
  for(int relay = 0; relay < 8; relay++) { // Boucle sur les 8 relais
    board.setRelayState(relay, true); // Active le relais
    delay(500); // Attends 100ms
    board.setRelayState(relay, false); // Désactive le relais
  }
*/
  // Lecture et affichage des entrées courant (4-20mA) et tension (0-10V).
  for (int i = 0; i < 4; i++) {
    float current = board.readAnalogmA(i); // Remplacez i par la broche ADC correspondante
    float voltage = board.readAnalogVoltage(i); // Idem
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
 Serial.println(digitalInputsBinary); // Affiche l'état des entrées numériques en format binaire de 8 bits

}