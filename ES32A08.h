#ifndef ES32A08_h
#define ES32A08_h

#include "Arduino.h"

// Définition des broches utilisées par les 3 resistres à décalage 74HC595D. Ceux-ci permettent de contrôler les 8 relais, ainsi que l'affichage sur les 4 digits.
#define DATA_PIN 13  // Broche de données pour les 74HC595D (affichage et relais)
#define LATCH_PIN 14 // Broche de verrouillage pour les 74HC595D
#define CLOCK_PIN 27 // Broche d'horloge pour les 74HC595D
#define OE_PIN 4     // Broche de sortie d'état (Output Enable) pour les 74HC595D

// Définition des broches utilisées par le 74HC165 (registre à décalage qui permet la lecture des 8 entrées numériques)
#define LOAD165_PIN 16 // Broche LOAD sur IO16 de l'ESP32
#define CLK165_PIN 17  // Broche CLK sur IO17 de l'ESP32
#define DATA165_PIN 5  // Broche DATA sur IO5 de l'ESP32

#define PWR_LED_PIN 15 // Broche pour la LED "PWR" sur la carte
#define LED_PIN 2 // Broche pour la LED intégrée sur l'ESP32


// Structure pour mapper les chiffres aux segments de l'afficheur 7 segments
struct DigitToSegment {
    char digit; // Caractère (chiffre)
    byte segment; // Code correspondant au segment
};

extern DigitToSegment digitToSegmentMap[]; // Tableau de correspondance des caractères vers mot binaire 8 bits.

// Classe pour gérer des délais sans bloquer l'exécution du programme
class NonBlockingDelay {
  private:
    unsigned long previousMillis; // Dernier enregistrement de temps
    unsigned long delayInterval; // Intervalle de délai
    bool running; // Si le délai est actif

  public:
    NonBlockingDelay(); // Constructeur
    void start(unsigned long interval); // Démarrer le délai
    bool isCompleted(); // Vérifier si le délai est terminé
    bool isRunning(); // Vérifier si le délai est en cours
};

// Classe principale pour la gestion de la carte ES32A08
class ES32A08 {
  public:
  
  void beginDisplayValue(const String &value, unsigned long updateDelay); // Démarre l'affichage avec la valeur spécifiée et le temps spécifié.
  
    uint8_t charToSegments(char c); // fonction qui récupérera le mot binaire correspondant au caractère à afficher (dans le tableau de correspondance).
	
	void afficherMessage(const char* message); // Fonction permettant de mettre a jour une variable de stockage du message à afficher sur le 4 digits. Cela permet d'appeler l'affichage à intervales réguliers sans ce soucier de l'écriture du message.
	
	void afficher(const char* message); // fonction d'affichage
    
    NonBlockingDelay delay1; // Premier délai non bloquant
    NonBlockingDelay delay2; // Second délai non bloquant
    NonBlockingDelay delay3; // Troisième délai non bloquant, si nécessaire et ainsi de suite.
	
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
    
	// Fonction pour contrôler la LED OnBoard "PWR". Pas nécessairement conditionné par une alimentation.
    void setPWRLED(bool state); 
	
	// Fonction pour mettre à jour l'état de la LED OnBoard "PWR". Utilisé pour faire clignoter.
    void updatePWRLED(); 
	
	// Fonction pour contrôler la LED OnBoard "PWR" en mode "Toggle" perpétuel. Cela agira en "watchdog" visuel et confirmera le bon déroulement du "loop()".
    void togglePWRLEDAsHeartbeat(bool enable);
	
    // Relay outputs
    void setRelayState(int relay, bool state); // Set relay state, relay 0-7, state: true=ON, false=OFF
	void setRelays(unsigned long relayStates); // Fonction de pilotage des 8 relais grâce à une suite de 8 bits, correspondant à l'état des 8 relais à piloter.
	
    bool readButton(int buttonNumber); // Lire l'état d'un bouton
	
	
	void clearDisplay(); // Fonction pour effacer l'affichage 4 Digits.
	void clearAll(); // Efface tous les états des relais et de l'affichage
    void sendToShiftRegister(byte segments, byte digits, byte relays); // Envoie des données aux registres à décalage
    void displayNumber(int number); // Affiche un nombre sur l'affichage 4 digits
    void setRelay(int relayNumber, bool state); // Contrôle l'état d'un relais spécifique
	
	void updateDisplay(byte digit, byte segments); // Met à jour et envoie les données pour l'affichage
	static const byte digitToSegment[11]; // Déclaration du tableau des segments correspondant aux chiffres.
	static const byte digitNumber[8]; // Déclaration du tableau mot 8 bits correspondant à quel digit.
	
void displayButtonPressed(int button); 	// Fonction pour afficher le bouton appuyé sur l'écran 4 digits


  private:
    
  NonBlockingDelay displayUpdateDelay; // Délai non bloquant pour l'actualisation de l'affichage dans la fonction ES32A08::beginDisplayValue(const String &value, unsigned long updateDelay)
  
  
    byte displayBuffer[4]; // Tableau pour stocker l'état des segments pour les 4 digits
    byte currentRelays; // État actuel des relais
    byte currentDigits; // Digit actuellement sélectionné pour l'affichage
    byte currentSegments; // Segments actuellement activés pour l'affichage
    void sendToShiftRegister(); // Méthode privée pour envoyer l'état à tous les registres
	
  void sendDataToShiftRegister(unsigned long data, int bitsCount);
  void beginRelays(); // Déclaration de la fonction d'initialisation des relais au démarrage de la carte.

  void sendToDisplay(byte data); // Fonction interne pour envoyer des données aux 74HC595 pour l'affichage 4 Digits notamment.
  
  uint8_t relayStates = 0; // Stocke l'état actuel des relais
  
  bool heartbeatEnabled; // Déclaration de la variable membre
  
  // Déclaration des pins pour les entrées analogiques
    const int mAInputPins[4] = {36, 39, 34, 35}; // In1 à In4 pour 4-20mA
    const int voltageInputPins[4] = {32, 33, 25, 26}; // V1 à V4 pour 0-10V
	
	// Déclarations des pins des boutons
    const int buttonPins[4] = {18, 19, 21, 23};
};

#endif
