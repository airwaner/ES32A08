#include "ES32A08.h"
#include <Wire.h> // Include Wire if needed for I2C communication, such as with a display

ES32A08::ES32A08() {
  // Constructor
}

void ES32A08::begin() {
// Configuration pour les relais	
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(PWR_LED_PIN, OUTPUT);
  setPWRLED(true); // Allume la LED "PWR" au démarrage
  digitalWrite(OE_PIN, LOW); // Active l'output du 74HC595D
  
// Configuration pour la lecture des entrées numériques
    pinMode(LOAD165_PIN, OUTPUT);
    pinMode(CLK165_PIN, OUTPUT);
    pinMode(DATA165_PIN, INPUT); 
    digitalWrite(LOAD165_PIN, HIGH); // Assurez-vous que le registre est prêt à charger les données

}

float ES32A08::readAnalogmA(int channel) {
    if (channel < 0 || channel > 3) return 0; // Validation de la plage du canal
    int adcValue = analogRead(mAInputPins[channel]); // Utilisez le numéro de broche ADC approprié
    float current = ((adcValue / 4095.0) * (20.0 - 4.0))*2.0692; // Converti en mA; avec un facteur de correction de la méthode de mesure de *2.0692
    return current;
}

float ES32A08::readAnalogVoltage(int channel) {
    if (channel < 0 || channel > 3) return 0; // Validation de la plage du canal
    int adcValue = analogRead(voltageInputPins[channel]);
    float voltage = ((adcValue / 4095.0) * 10.0)*2.0692; // Convertit en volts, pour une plage de 0 à 10V (*2 pour facteur de correction.)
    return voltage;
}


void ES32A08::setRelayState(int relay, bool state) {
  bitWrite(relayStates, relay, state); // écriture de l'état du relais sélectionné.
  sendDataToShiftRegister(relayStates);
}

void ES32A08::sendDataToShiftRegister(uint8_t data) {
  digitalWrite(LATCH_PIN, LOW);
  for (int i = 7; i >= 0; i--) { // Envoyer les bits de MSB à LSB
    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(DATA_PIN, bitRead(data, i) ? HIGH : LOW);
    digitalWrite(CLOCK_PIN, HIGH); // Clock in le bit
  }
  digitalWrite(LATCH_PIN, HIGH); // Transfère les données vers les sorties
}
uint8_t ES32A08::readDigitalInputs() {
    uint8_t inputs = 0;
    digitalWrite(LOAD165_PIN, LOW);  // Préparez le 74HC165 pour la capture des entrées
    delayMicroseconds(5);         // Un petit délai pour stabiliser le signal
    digitalWrite(LOAD165_PIN, HIGH); // Capturez les entrées dans le registre

    for (int i = 0; i < 8; i++) { // Lisez les 8 bits
        digitalWrite(CLK165_PIN, LOW); // Initiez un cycle de clock
        delayMicroseconds(5);       // Un petit délai pour le timing du clock
        bitWrite(inputs, 7 - i, digitalRead(DATA165_PIN)); // Lisez le bit actuel
        digitalWrite(CLK165_PIN, HIGH); // Terminez le cycle de clock
    }

    return inputs; // Retournez l'état des 8 entrées numériques
}

bool ES32A08::getDigitalInputState(int inputNumber) {
    if (inputNumber < 1 || inputNumber > 8) return false; // Validation de l'entrée
    uint8_t digitalInputs = readDigitalInputs(); // Nécessite la fonction juste au dessus.
    return digitalInputs & (1 << (inputNumber - 1)); // Renvoie l'état de l'entrée spécifiée
}

String ES32A08::getFormattedDigitalInputs() {
    uint8_t digitalInputs = readDigitalInputs(); // Utilisez la fonction existante pour lire les états
    String binaryStr = "";
    for (int i = 7; i >= 0; i--) {
        binaryStr += (digitalInputs & (1 << i)) ? "1" : "0";
    }
    return binaryStr; // Retourner la chaine de caractères avec le mot sous forme de Binaire 8 bits.
}


void ES32A08::setPWRLED(bool state) {
    // Configure la LED pour s'allumer avec un état logique haut
    digitalWrite(PWR_LED_PIN, !state); // Inversion de l'état pour correspondre au câblage physique
}
