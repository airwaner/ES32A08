#include "ES32A08.h"

// Constructeur et méthodes de NonBlockingDelay
NonBlockingDelay::NonBlockingDelay() : previousMillis(0), delayInterval(0), running(false) {}

void NonBlockingDelay::start(unsigned long interval) {
    this->delayInterval = interval;
    this->previousMillis = millis();
    this->running = true;
}

bool NonBlockingDelay::isCompleted() {
    if (!this->running) return false;
    unsigned long currentMillis = millis();
    if (currentMillis - this->previousMillis >= this->delayInterval) {
        this->running = false; // Arrête le délai
        return true;
    }
    return false;
}

bool NonBlockingDelay::isRunning() {
    return this->running;
}

//-------------------------------------------------------------------------

ES32A08::ES32A08() {
	// Initialise displayBuffer à 0
    memset(displayBuffer, 0, sizeof(displayBuffer));
  // Constructor
}

void ES32A08::begin() {
// Configuration pour les relais	
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  
  // On écrtit 0 aux broches de contrôles des registres à décalage 74HC595D (cablés en série) afin d'éviter toute instabilité au démarrage.
  digitalWrite(DATA_PIN, LOW); // Active l'output du 74HC595D
  digitalWrite(LATCH_PIN, LOW); // Active l'output du 74HC595D
  digitalWrite(CLOCK_PIN, LOW); // Active l'output du 74HC595D
  digitalWrite(OE_PIN, LOW); // Active l'output du 74HC595D
  
  // Réinitialisation des registres
    clearAll();
	
  pinMode(PWR_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT); // Définition de la broche Led BLEUE comme sortie

  pinMode(buttonPins[0], INPUT_PULLUP); // Configurer les boutons en entrée avec pull-up
  pinMode(buttonPins[1], INPUT_PULLUP); // Configurer les boutons en entrée avec pull-up
  pinMode(buttonPins[2], INPUT_PULLUP); // Configurer les boutons en entrée avec pull-up
  pinMode(buttonPins[3], INPUT_PULLUP); // Configurer les boutons en entrée avec pull-up
  
  // Fixer l'état des relais éteints au démarrage
  beginRelays(); //Forcer les relais à l'état bas au démarrage de la carte.
   
// Configuration pour la lecture des entrées numériques
    pinMode(LOAD165_PIN, OUTPUT);
    pinMode(CLK165_PIN, OUTPUT);
    pinMode(DATA165_PIN, INPUT); 
    digitalWrite(LOAD165_PIN, HIGH); // Assurez-vous que le registre est prêt à charger les données
	
}

void ES32A08::clearAll() {
	currentRelays = 0;
	currentDigits = 0;
	currentSegments = 0;
    // Méthode pour réinitialiser tous les registres à 0
    sendToShiftRegister();
} // Fonction de réinitialisation des registres à décalage 74HC595D. Sert à éviter des états inconnus où instabilités au démarrage.

// Pour stocker l'état actuel des relais
byte currentRelayState = 0; // Chaque bit représentera l'état d'un relais

// Exemple pour les chiffres de 0 à 9 en utilisant les segments a-g plus le point décimal (supposons que '0' éteint le segment et '1' l'allume)
const byte ES32A08::digitToSegment[11] = {
    0b00111111, // Chiffre 0
    0b00000110, // Chiffre 1
    0b01011011, // Chiffre 2
    0b01001111, // Chiffre 3
    0b01100110, // Chiffre 4
    0b01101101, // Chiffre 5
    0b01111101, // Chiffre 6
    0b00000111, // Chiffre 7
    0b01111111, // Chiffre 8
    0b01101111, // Chiffre 9
	0b10000000  // Point Décimal .
};
// Tableau contenant les adresses 8bits des digits.
const byte ES32A08::digitNumber[8] = {
	0b11111111, // Position aucun digit.
    0b11111110, // Position du digit 1.
    0b11111101, // Position du digit 2.
    0b11111011, // Position du digit 3.
    0b11110111, // Position du digit 4.
	0b11111100, // Position du digit 1+2.
	0b11111000, // Position du digit 1+2+3.
	0b11110000 // Position des digit 1+2+3+4.
};

// Initialisation du tableau de correspondances
DigitToSegment digitToSegmentMap[] = {
    {'0', 0b00111111},
    {'1', 0b00000110},
	{'2', 0b01011011}, 
    {'3', 0b01001111},
    {'4', 0b01100110},
    {'5', 0b01101101},
    {'6', 0b01111101},
    {'7', 0b00000111},
    {'8', 0b01111111},
    {'9', 0b01101111},
	{'.', 0b10000000}
};

void ES32A08::sendToShiftRegister() {
// Inversez les bits du paramètre `digits` pour que '0' active le digit
    //digits = ~digits; // Cela inverse tous les bits; un '0' devient un '1' et vice versa. Essai car les digits restent noirs.
 	//segments = ~segments; // Inverse aussi pour activer les segments avec un '0'
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentRelays);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentDigits);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentSegments);
    digitalWrite(LATCH_PIN, HIGH);
}

void ES32A08::updateDisplay(byte digit, byte segments){
	currentDigits = digit; // Adaptez en fonction de votre logique
    currentSegments = segments;
    sendToShiftRegister(); // Met à jour l'affichage
    // Note: Assurez-vous que toute logique ici est conçue pour être exécutée après l'intervalle spécifié
}

void ES32A08::displayButtonPressed(int button) {
    
    // Affiche le chiffre correspondant au bouton appuyé
    updateDisplay(~(button), digitToSegment[button]);
}

uint8_t ES32A08::charToSegments(char c) {
    switch(c) {
        case '0': return 0b00111111;
        case '1': return 0b00000110;
        case '2': return 0b01011011;
		case '3': return 0b01001111;
        case '4': return 0b01100110;
        case '5': return 0b01101101;
		case '6': return 0b01111101;
        case '7': return 0b00000111;
        case '8': return 0b01111111;
		case '9': return 0b01101111;
        case '.': return 0b10000000;
        case ' ': return 0b00000000;
		case 'A': return 0b01110111;
        // Ajoutez les autres cas ici pour chaque caractère
        default:  return 0b00000000; // Retourne 0 pour les caractères non reconnus
    }
}

void ES32A08::afficher(const char* message) {
    for(int i = 0; message[i] != '\0' && i < 4; i++) {
        // Convertit chaque caractère en segments
        uint8_t segments = charToSegments(message[i]);
        // Affiche le caractère sur le digit correspondant
        updateDisplay(digitNumber[i+1], segments); // Cette fonction doit être implémentée pour gérer l'affichage
		delayMicroseconds(500); // un délai bloquant obligatoire car comme chaque programme est unique, il n'est pas possible de gérer efficacement le temps de cycle programme. Cela influence donc la lisibilité des chiffres sont l'on utilise des délais non bloquants. Choix à été fait de passer par un µdélai pour impacter le moins possible le reste du programme.Varier entre 1 et 500µS pour changer la luminosité.
    }
	// On éteint tous les digits une fois les 4 digits affichés 1 par 1 afin que le dernier ne soit pas plus visible que les autres à cause du temps d'éxécution du reste du programme.
	currentDigits = 0b11111111; // Désélectionne tous les digits 
    currentSegments = 0b00000000; // Éteint tous les segments
    sendToShiftRegister(); // Applique l'état d'éteignement
	delayMicroseconds(1); // Temps pour fixer l'état d'exctinction. Plus ce délai est long, plus la chance de rendre perceptible l'extinction augmentera en fonction de la durée de fonctionnement du reste du programme. Jusque 25mS de temps programme total, le résultat est acceptable et lisible !
}

void ES32A08::beginDisplayValue(const String &value, unsigned long updateDelay) {
    displayUpdateDelay.start(updateDelay); // Configure le délai avec la valeur spécifiée
    
    // Si la valeur est numérique et inférieure à 10000, formatez-la pour supprimer les zéros non significatifs
    if (value.toInt() < 10000 && value.toInt() > 0) {
        char buffer[5]; // Assez grand pour n'importe quel nombre à 4 chiffres + terminateur null
        sprintf(buffer, "%d", value.toInt()); // Convertit en chaîne sans zéros non significatifs sur la gauche
        afficher(buffer); // Affiche la valeur
    } else {
        // Pour les chaînes de caractères ou les nombres ≥ 10000, affiche directement la valeur
        afficher(value.c_str()); // Affiche la chaîne telle quelle
    }
}


void ES32A08::setRelay(int relay, bool state) {
    if(state)
        relayStates |= (1 << (relay - 1)); // Met à 1 le bit correspondant au relais
    else
        relayStates &= ~(1 << (relay - 1)); // Met à 0 le bit correspondant au relais
	currentRelays = relayStates; // Stocker dans "currentRelays" afin d'avoir une référence lors de futures écritures vers les registres. On garde ainsi l'état des relais tout en mettant à jour l'afficheur.
	
    sendToShiftRegister(); // Mettez à jour les registres à décalage ici
}// Fonction d'écriture vers les relais.

// Fonction de lecture d'un canal d'entrée courant (4-20mA).
float ES32A08::readAnalogmA(int channel) {
    if (channel < 0 || channel > 3) return 0; // Validation de la plage du canal
    int adcValue = analogRead(mAInputPins[channel]); // Utilisez le numéro de broche ADC approprié
    float current = ((adcValue / 4095.0) * (20.0 - 4.0))*2.0692; // Converti en mA; avec un facteur de correction de la méthode de mesure de *2.0692
    return current;
}

// Fonction de lecture d'un canal d'entrée tension (0-10V).
float ES32A08::readAnalogVoltage(int channel) {
    if (channel < 0 || channel > 3) return 0; // Validation de la plage du canal
    int adcValue = analogRead(voltageInputPins[channel]);
    float voltage = ((adcValue / 4095.0) * 10.0)*2.0692; // Convertit en volts, pour une plage de 0 à 10V (*2 pour facteur de correction.)
    return voltage;
}


void ES32A08::setRelayState(int relay, bool state) {
  bitWrite(relayStates, relay, state); // écriture de l'état du relais sélectionné.
  sendToShiftRegister(); // Envoi au registre à décalage des ordres de relais.
}

void ES32A08::setRelays(unsigned long relayStates) {
	currentRelays = relayStates;  // On met à jour les 8 bits d'état des 8 relais. 1 allumé, 0 éteint. Position de 1 à 8. 
  sendToShiftRegister(); // Envoyez les données avec les 24 bits si vous avez 3 registres
} // Permet le contrôle des 8 relais en une seule fonction par un 8 bits.


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

void ES32A08::togglePWRLEDAsHeartbeat(bool enable) {
    heartbeatEnabled = enable;
} // Configure la LED en tant que preuve du déroulement du programme. Celle-ci change d'état à chaque itération de la boucle programme.

void ES32A08::updatePWRLED() {
    if(heartbeatEnabled) {
        static bool ledState = false;
        setPWRLED(ledState);
		//digitalWrite(LED_PIN, !ledState); // Allumer la LED BLEUE de l'Esp32

        ledState = !ledState; // Change l'état pour la prochaine itération
    }
} // Configure la LED en tant que preuve du déroulement du programme. Celle-ci change d'état à chaque itération de la boucle programme.

bool ES32A08::readButton(int buttonNumber) {
    return !digitalRead(buttonPins[buttonNumber-1]); // Lire et inverser l'état (bouton pressé = LOW)
}

void ES32A08::beginRelays()
{
	for(int relay = 0; relay < 8; relay++) { // Boucle sur les 8 relais
    setRelays(00000000); // Désactive le relais
}
}
