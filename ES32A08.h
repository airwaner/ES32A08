#ifndef ES32A08_h
#define ES32A08_h

#include "Arduino.h"

// Définition des broches utilisées par les 3 resistres à décalage 74HC595D. Ceux-ci permettent de contrôler les 8 relais, ainsi que l'affichage sur les 4 digits.
#define DATA_PIN 13  // Correspond à IO13 sur ESP32
#define LATCH_PIN 14 // Correspond à IO14 sur ESP32
#define CLOCK_PIN 27 // Correspond à IO27 sur ESP32
#define OE_PIN 4     // Correspond à IO04 sur ESP32

// Définition des broches utilisées par le 74HC165 (registre à décalage qui permet la lecture des 8 entrées numériques)
#define LOAD165_PIN 16 // Broche LOAD sur IO16 de l'ESP32
#define CLK165_PIN 17  // Broche CLK sur IO17 de l'ESP32
#define DATA165_PIN 5  // Broche DATA sur IO5 de l'ESP32

#define PWR_LED_PIN 15 // Définition de la broche pour la LED "PWR"

class ES32A08 {
  public:
    ES32A08(); // Constructor
    void begin(); // Initialize the board

    // Analog inputs
	float readAnalogmA(int channel); // Pour lire les entrées 4-20mA
    float readAnalogVoltage(int channel); // Pour lire les entrées 0-10V

    // Digital inputs and buttons
    bool readDigitalInput(int pin); // Read digital input state
	bool getDigitalInputState(int inputNumber); // Lire une entrée numérique en particulier (indiquer le numéro de 1 à 8) 
	uint8_t readDigitalInputs();
	String getFormattedDigitalInputs(); // Fonction pour obtenir les entrées numériques formatées sur un 8 bits afin de voir directement l'état des 8 entrées sur un binaire 8 bits.
    bool readButtonState(int button); // Read button state, button 0-3

    // Display control
    void displayNumber(int number); // Display a number on the 4-digit screen

	// Fonction pour contrôler la LED OnBoard "PWR". Pas nécessairement conditionné par une alimentation.
    void setPWRLED(bool state); 
	
    // Relay outputs
    void setRelayState(int relay, bool state); // Set relay state, relay 0-7, state: true=ON, false=OFF
	
	// Boutons Key 1 --> Key 4 sous les 4 Digits.
	 void beginButtons(); // Initialiser les boutons
    bool readButton(int buttonNumber); // Lire l'état d'un bouton
	
	
  private:
  
  void sendDataToShiftRegister(uint8_t data);
  uint8_t relayStates = 0; // Stocke l'état actuel des relais
  
  // Déclaration des pins pour les entrées analogiques
    const int mAInputPins[4] = {36, 39, 34, 35}; // In1 à In4 pour 4-20mA
    const int voltageInputPins[4] = {32, 33, 25, 26}; // V1 à V4 pour 0-10V
	
	// Déclarations des pins des boutons
    const int buttonPins[4] = {18, 19, 21, 23};
    // Private variables and functions here
};

#endif
