#include "ES32A08.h"

// Constructeur : initialise les membres de la classe avec des valeurs par défaut.
NonBlockingDelay::NonBlockingDelay() : previousMillis(0), delayInterval(0), running(false) {}

// Démarre le délai non bloquant avec un intervalle spécifié.
void NonBlockingDelay::start(unsigned long interval) {
    this->delayInterval = interval; // Définit l'intervalle du délai.
    this->previousMillis = millis(); // Enregistre le temps actuel comme point de départ du délai.
    this->running = true; // Indique que le délai est maintenant actif.
}

// Vérifie si le délai est complété.
bool NonBlockingDelay::isCompleted() {
    if (!this->running) return false; // Si le délai n'est pas actif, retourne faux immédiatement.
    unsigned long currentMillis = millis(); // Obtient le temps actuel.
	// Vérifie si l'intervalle de délai s'est écoulé.
    if (currentMillis - this->previousMillis >= this->delayInterval) {
        this->running = false; // Arrête le délai
        return true; // Retourne vrai, indiquant que le délai est complété.
    }
    return false; // Retourne faux, le délai est toujours en cours.
}

// Vérifie si le délai est actuellement actif.
bool NonBlockingDelay::isRunning() {
    return this->running; // Retourne l'état actif du délai.
}

//-------------------------------------------------------------------------

ES32A08::ES32A08() {
	// Initialise displayBuffer à 0
    memset(displayBuffer, 0, sizeof(displayBuffer));
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
	digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentRelays);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentDigits);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, currentSegments);
    digitalWrite(LATCH_PIN, HIGH);
} // Envoi vers les registres à décalage 74HC595D. Nécessite d'envoyer le flot complet de 3 bytes pour ne pas imposer d'état inconnu à quelconque registre. On envoie donc ici en série 24 bits.

void ES32A08::updateDisplay(byte digit, byte segments){
	currentDigits = digit; // Copie de l'argument digit dans le byte currentDigits
    currentSegments = segments; // Copie de l'argument segments dans le byte currentSegments
    sendToShiftRegister(); // Envoi vers les registres à décalage.
    // Note: Assurez-vous que toute logique ici est conçue pour être exécutée après l'intervalle spécifié
}

void ES32A08::displayButtonPressed(int button) {
    
    updateDisplay(~(button), digitToSegment[button]);
}    // Affiche le chiffre correspondant au bouton appuyé à son emplacement.


// Tableau de correspondance des caractères en binaire 8 bits. Le plus à gauche est le DP. Segments abcdefg en partant de la droite.
uint8_t ES32A08::charToSegments(char c) {
    switch(c) {
        case '0': return 0b00111111; // 0
        case '1': return 0b00000110; // 1
        case '2': return 0b01011011; // 2
        case '3': return 0b01001111; // 3
        case '4': return 0b01100110; // 4
        case '5': return 0b01101101; // 5 ou S
        case '6': return 0b01111101; // 6
        case '7': return 0b00000111; // 7
        case '8': return 0b01111111; // 8
        case '9': return 0b01101111; // 9
        case ' ': return 0b00000000; // espace
        case '.': return 0b10000000; // point décimal
        case '-': return 0b01000000; // tiret
        case '_': return 0b00001000; // Underscore
		case 'A': return 0b01110111; // A majuscule
        case 'a': return 0b01011111; // a minuscule, différent de A
        case 'B': case 'b': return 0b01111100; // b minuscule car B trop proche de 8.
        case 'C': return 0b00111001; // C majuscule
        case 'c': return 0b01011000; // c minuscule, différent de C
        case 'D': case 'd': return 0b01011110; // D similaire à d pour éviter la confusion avec O
        case 'E': return 0b01111001; // E majuscule,
        case 'e': return 0b01111011; // e minuscule,
        case 'F': case 'f': return 0b01110001; // F similaire à f
        case 'G': case 'g': return 0b01101111; // g minuscule, choisie pour G car plus claire, évitant la confusion avec 9
        case 'H': return 0b01110110; // H majuscule
        case 'h': return 0b01110100; // h minuscule, différent de H
        case 'I': return 0b00000110; // I majuscule, identique à 1
        case 'i': return 0b00010000; // i minuscule, point central
        case 'J': case 'j': return 0b00011110; // j minuscule, identique à J pour la clarté        
        case 'L': return 0b00111000; // L majuscule
        case 'l': return 0b00011000; // l minuscule, différent de L
        case 'M': case 'm': return 0b00110111; // M majuscule, et m minuscule, même représentation
        case 'N': case 'n': return 0b01010100; // N majuscule et n minuscule, même représentation
        case 'O': case 'o': return 0b01011100; // o minuscule uniquement, sinon trop proche de Zéro.
        case 'P': case 'p': return 0b01110011; // p minuscule, identique à P, pas de représentation distincte nécessaire
        case 'Q': case 'q': return 0b01100111; // q minuscule, uniquement
        case 'R': case 'r': return 0b01010000; // r minuscule, uniquement.
        case 'S': case 's': return 0b01101101; // S majuscule , identique à 5, utilisée pour S
        case 'T': case 't': return 0b01111000; // t minuscule
        case 'U': return 0b00111110; // U majuscule
        case 'u': return 0b00011100; // u minuscule, différent de U
        case 'Y': case 'y': return 0b01100110; // y minuscule, identique à Y, pas de représentation distincte nécessaire
        case 'Z': case 'z': return 0b01011011; // z minuscule, identique à 2, utilisée pour Z

        // Ajouter les autres cas ici pour ajouter d'autres caractères.
        default:  return 0b00000000; // Retourne 0 pour les caractères non reconnus
    }
}


// Fonction d'affichage pour les chaines de caractères:
void ES32A08::afficher(const char* message) {
    for(int i = 0; message[i] != '\0' && i < 4; i++) {
        // Convertit chaque caractère en segments
        uint8_t segments = charToSegments(message[i]);
        // Affiche le caractère sur le digit correspondant
        updateDisplay(digitNumber[i+1], segments); // Cette fonction doit être implémentée pour gérer l'affichage
		delayMicroseconds(2000); // un délai bloquant obligatoire car comme chaque programme est unique, il n'est pas possible de gérer efficacement le temps de cycle programme. Cela influence donc la lisibilité des chiffres sont l'on utilise des délais non bloquants. Choix à été fait de passer par un µdélai pour impacter le moins possible le reste du programme.Varier entre 1 et 500µS pour changer la luminosité.
    }
	// On éteint tous les digits une fois les 4 digits affichés 1 par 1 afin que le dernier ne soit pas plus visible que les autres à cause du temps d'éxécution du reste du programme.
	currentDigits = 0b11111111; // Désélectionne tous les digits 
    currentSegments = 0b00000000; // Éteint tous les segments
    sendToShiftRegister(); // Applique l'état d'éteignement
	delayMicroseconds(0); // Temps pour fixer l'état d'exctinction. Plus ce délai est long, plus la chance de rendre perceptible l'extinction augmentera en fonction de la durée de fonctionnement du reste du programme. Jusque 25mS de temps programme total, le résultat est acceptable et lisible !
}

// Fonction d'affichage pour les entiers:
void ES32A08::afficher(int number) {
	
	
    if (number > 9999 || number < -999) {
        // Dépassement de capacité, on affiche des tirets
        afficher(" -- ");
    } else {
		char buffer[5]; // Buffer pour contenir le nombre + caractère de fin de chaîne
    snprintf(buffer, sizeof(buffer), "%4d", number); // Formatte le nombre en 4 chiffres, complétés par des espaces si nécessaire

    for (int i = 0; i < 4; i++) {
        if (buffer[i] == ' ') {
            // Si le caractère est un espace, éteindre les segments pour ce digit
            updateDisplay(digitNumber[i + 1], 0x00); // Assurez-vous que cette ligne correspond à la manière dont votre bibliothèque éteint un digit
        } else {
            // Convertit le caractère en segments et met à jour l'affichage pour le digit courant
            updateDisplay(digitNumber[i + 1], charToSegments(buffer[i]));
        }
        delayMicroseconds(2000); // Délai pour la persistance rétinienne, remplacez si nécessaire par une approche non bloquante
    }

    // Éteint tous les segments après le dernier digit affiché pour éviter qu'il reste allumé plus longtemps que les autres
    currentDigits = 0b11111111 ; // Sélectionne tous les digit pour éteindre l'affichage
    currentSegments = 0b00000000 ; // Éteint tous les segments
    sendToShiftRegister();
    delayMicroseconds(0); // Délai très court pour fixer l'état d'extinction, ajuster en fonction des besoins.
    }  
}

void ES32A08::afficher(float number) {
    // Étape 1: Vérification de la capacité d'affichage
    int numIntPart = int(number); // Partie entière du nombre
    float numDecPart = number - numIntPart; // Partie décimale du nombre
    int numIntPartDigits = numIntPart > 0 ? (int)log10((double)numIntPart) + 1 : 1; // Nombre de chiffres dans la partie entière
    int decimalPlaces = numIntPartDigits >= 4 ? 0 : 3 - numIntPartDigits; // Calcul des décimales possibles

    // Si la partie entière dépasse les 4 chiffres ou nécessite plus de l'espace disponible pour les décimales
    if (numIntPartDigits > 4 || (numIntPartDigits == 4 && numDecPart > 0)) {
        // Affiche " -- " pour indiquer l'impossibilité de l'affichage correct du nombre
        afficher("--");
        return;
    }

    // Étape 2: Conversion du nombre en chaîne avec les décimales appropriées
    char buffer[6]; // Buffer pour le nombre converti en chaîne
    dtostrf(number, 4, decimalPlaces, buffer);

    // Étape 3: Suppression des espaces blancs initiaux pour les nombres < 1
    char *trimmedBuffer = buffer;
    while (*trimmedBuffer == ' ') {
        trimmedBuffer++;
    }

    // Étape 4: Inversion de l'ordre des caractères pour l'affichage
    char reversedBuffer[5] = {' ', ' ', ' ', ' ', '\0'};
    int trimmedLen = strlen(trimmedBuffer);
    for (int i = 0; i < trimmedLen; i++) {
        reversedBuffer[i] = trimmedBuffer[trimmedLen - 1 - i];
    }

    // Étape 5: Affichage en gérant spécialement le point décimal
    for (int i = 0; i < 4; i++) {
        char c = reversedBuffer[i];
        uint8_t segments;
        
        if (c == '.') {
            if (i < 3 && reversedBuffer[i + 1] != ' ') {
                segments = charToSegments(reversedBuffer[i + 1]) | 0b10000000;
                reversedBuffer[i + 1] = ' '; // Marque le chiffre comme traité
            } else {
                segments = 0b10000000;
            }
        } else if (c != ' ') {
            segments = charToSegments(c);
        } else {
            segments = 0; // Éteint le segment pour l'espace
        }

        updateDisplay(digitNumber[4 - i], segments);
        delayMicroseconds(2000);
    }

    // Étape 6: Éteindre tous les segments après l'affichage
    currentDigits = 0b11111111;
    currentSegments = 0b00000000;
    sendToShiftRegister();
    delayMicroseconds(0);
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
