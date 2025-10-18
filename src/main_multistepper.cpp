#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =========== PIN DEFINITIONS ===========
// DISPLAY SH1107 (I2C)
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_ADDRESS 0x3C

// STEPPER 1 (X)
#define STEP_PIN_X 14
#define DIR_PIN_X 27
#define ENABLE_PIN_X 26

// STEPPER 2 (Y)
#define STEP_PIN_Y 25
#define DIR_PIN_Y 33
#define ENABLE_PIN_Y 32

// LIMIT SWITCHES
#define LIMIT_X_MIN 34
#define LIMIT_X_MAX 35
#define LIMIT_Y_MIN 36
#define LIMIT_Y_MAX 39

// PULSANTI
#define BUTTON_START 12
#define BUTTON_STOP 13
#define BUTTON_PAUSE 5

// POTENZIOMETRI
#define POTENZIOMETRO_MOTORE1 15
#define POTENZIOMETRO_MOTORE2 4

// RELE'
#define RELE_PIN 3

// =========== STATI SISTEMA ===========
enum SystemState
{
    STATE_OFF,
    STATE_HOMING,
    STATE_READY,
    STATE_RUNNING,
    STATE_PAUSED,
    STATE_EMERGENCY
};
SystemState currentState = STATE_OFF;

// =========== VARIABILI GLOBALI ===========
// Debouncing
unsigned long lastDebounceTimeStart = 0;
unsigned long lastDebounceTimeStop = 0;
unsigned long lastDebounceTimePause = 0;
const unsigned long debounceDelay = 50;

int buttonStartState = HIGH;
int lastButtonStartState = HIGH;
int buttonStopState = HIGH;
int lastButtonStopState = HIGH;
int buttonPauseState = HIGH;
int lastButtonPauseState = HIGH;

bool startPressed = false;
bool stopPressed = false;
bool pausePressed = false;

// Ciclo coordinato
bool cicloAttivo = false;
bool motore1Completato = false;
bool motore2Completato = false;
unsigned long tempoInizioCiclo = 0;

// Potenziometri
int velocitaMotore1 = 1000;
int velocitaMotore2 = 800;
int ultimaLetturaPot1 = 0;
int ultimaLetturaPot2 = 0;
unsigned long ultimoAggiornamentoPot = 0;
const unsigned long INTERVALLO_POT = 100;
const int VELOCITA_MIN = 100;
const int VELOCITA_MAX = 3000;

// Stato motori e relè
bool motoriAbilitati = false;
bool releAttivo = false;

// Variabili per homing
bool homingInCorso = false;
bool homingXCompletato = false;
bool homingYCompletato = false;
unsigned long homingStartTime = 0;

// =========== OGGETTI GLOBALI ===========
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
MultiStepper steppers;
Adafruit_SH1107 display = Adafruit_SH1107(128, 128, &Wire);

// Array per posizioni MultiStepper
long targetPositions[2] = {0, 0};

// =========== PROTOTIPI FUNZIONI ===========
void emergencyStop();
void checkButtons();
void aggiornaPotenziometri();
void professionalHoming();
void updateDisplay();
void setupDisplay();
void cicloCoordinato();
void controllaMotore2();
void fermaTutto();
void pauseCycle();
void resumeCycle();
void attivaRele();
void disattivaRele();
void abilitaMotori();
void disabilitaMotori();
void debugRele();

// =========== SETUP DISPLAY ===========
void setupDisplay()
{
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!display.begin(OLED_ADDRESS, true))
    {
        Serial.println("SH1107 non trovato!");
        while (1)
            ;
    }

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    display.display();

    Serial.println("Display SH1107 inizializzato");
}

// =========== GESTIONE MOTORI ===========
void abilitaMotori()
{
    if (!motoriAbilitati)
    {
        digitalWrite(ENABLE_PIN_X, LOW);
        digitalWrite(ENABLE_PIN_Y, LOW);
        motoriAbilitati = true;
        Serial.println("Motori abilitati");
    }
}

void disabilitaMotori()
{
    if (motoriAbilitati)
    {
        digitalWrite(ENABLE_PIN_X, HIGH);
        digitalWrite(ENABLE_PIN_Y, HIGH);
        motoriAbilitati = false;
        Serial.println("Motori disabilitati");
    }
}

// =========== GESTIONE RELE ===========
void attivaRele()
{
    if (!releAttivo)
    {
        digitalWrite(RELE_PIN, HIGH);
        releAttivo = true;
        Serial.println("Rele attivato");
    }
}

void disattivaRele()
{
    if (releAttivo)
    {
        digitalWrite(RELE_PIN, LOW);
        releAttivo = false;
        Serial.println("Rele disattivato");
    }
}

// =========== SETUP ===========
void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Display prima di tutto
    setupDisplay();

    // Configurazione pin STEP/DIR
    pinMode(STEP_PIN_X, OUTPUT);
    pinMode(DIR_PIN_X, OUTPUT);
    pinMode(STEP_PIN_Y, OUTPUT);
    pinMode(DIR_PIN_Y, OUTPUT);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(DIR_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    digitalWrite(DIR_PIN_Y, LOW);

    display.setTextSize(2);
    display.setCursor(10, 50);
    display.print("AVVIO...");
    display.display();
    delay(1000);

    // Stepper enable
    pinMode(ENABLE_PIN_X, OUTPUT);
    pinMode(ENABLE_PIN_Y, OUTPUT);
    disabilitaMotori();

    // Pulsanti
    pinMode(BUTTON_START, INPUT_PULLUP);
    pinMode(BUTTON_STOP, INPUT_PULLUP);
    pinMode(BUTTON_PAUSE, INPUT_PULLUP);

    // Finecorsa
    pinMode(LIMIT_X_MIN, INPUT_PULLUP);
    pinMode(LIMIT_X_MAX, INPUT_PULLUP);
    pinMode(LIMIT_Y_MIN, INPUT_PULLUP);
    pinMode(LIMIT_Y_MAX, INPUT_PULLUP);

    // Relè
    pinMode(RELE_PIN, OUTPUT);
    disattivaRele();

    // ⭐ CONFIGURAZIONE MULTISTEPPER
    steppers.addStepper(stepperX);
    steppers.addStepper(stepperY);

    // Configurazione stepper
    stepperX.setMaxSpeed(velocitaMotore1);
    stepperY.setMaxSpeed(velocitaMotore2);
    stepperX.setAcceleration(2000);
    stepperY.setAcceleration(2000);

    currentState = STATE_OFF;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 50);
    display.print("PRONTO");
    display.display();

    Serial.println("Sistema pronto con MultiStepper");
}

// =========== EMERGENCY STOP ===========
void emergencyStop()
{
    stepperX.stop();
    stepperY.stop();
    disabilitaMotori();
    disattivaRele();
    cicloAttivo = false;
    currentState = STATE_EMERGENCY;
    Serial.println("FERMO EMERGENZA");
}

// =========== GESTIONE PULSANTI ===========
void checkButtons()
{
    int currentStart = digitalRead(BUTTON_START);
    int currentStop = digitalRead(BUTTON_STOP);
    int currentPause = digitalRead(BUTTON_PAUSE);

    startPressed = false;
    stopPressed = false;
    pausePressed = false;

    // DEBOUNCING START
    if (currentStart != lastButtonStartState)
    {
        lastDebounceTimeStart = millis();
    }
    if ((millis() - lastDebounceTimeStart) > debounceDelay)
    {
        if (currentStart != buttonStartState)
        {
            buttonStartState = currentStart;
            if (buttonStartState == LOW)
                startPressed = true;
        }
    }
    lastButtonStartState = currentStart;

    // DEBOUNCING STOP
    if (currentStop != lastButtonStopState)
    {
        lastDebounceTimeStop = millis();
    }
    if ((millis() - lastDebounceTimeStop) > debounceDelay)
    {
        if (currentStop != buttonStopState)
        {
            buttonStopState = currentStop;
            if (buttonStopState == LOW)
                stopPressed = true;
        }
    }
    lastButtonStopState = currentStop;

    // DEBOUNCING PAUSE
    if (currentPause != lastButtonPauseState)
    {
        lastDebounceTimePause = millis();
    }
    if ((millis() - lastDebounceTimePause) > debounceDelay)
    {
        if (currentPause != buttonPauseState)
        {
            buttonPauseState = currentPause;
            if (buttonPauseState == LOW)
                pausePressed = true;
        }
    }
    lastButtonPauseState = currentPause;

    if (stopPressed)
    {
        emergencyStop();
        return;
    }

    switch (currentState)
    {
    case STATE_OFF:
        if (startPressed)
        {
            currentState = STATE_HOMING;
            homingInCorso = true;
            homingXCompletato = false;
            homingYCompletato = false;
            homingStartTime = millis();
            abilitaMotori();
        }
        break;
    case STATE_READY:
        if (startPressed)
        {
            currentState = STATE_RUNNING;
            abilitaMotori();
        }
        break;
    case STATE_PAUSED:
        if (startPressed)
        {
            resumeCycle();
            abilitaMotori();
        }
        break;
    case STATE_RUNNING:
        if (pausePressed)
        {
            pauseCycle();
            disabilitaMotori();
        }
        break;
    case STATE_EMERGENCY:
        if (startPressed)
        {
            currentState = STATE_OFF;
            disabilitaMotori();
        }
        break;
    }
}

// =========== GESTIONE POTENZIOMETRI ===========
void aggiornaPotenziometri()
{
    if (millis() - ultimoAggiornamentoPot > INTERVALLO_POT)
    {
        int letturaPot1 = analogRead(POTENZIOMETRO_MOTORE1);
        int letturaPot2 = analogRead(POTENZIOMETRO_MOTORE2);

        if (abs(letturaPot1 - ultimaLetturaPot1) > 50)
        {
            velocitaMotore1 = map(letturaPot1, 0, 4095, VELOCITA_MIN, VELOCITA_MAX);
            ultimaLetturaPot1 = letturaPot1;
            if (currentState == STATE_READY || currentState == STATE_RUNNING)
            {
                stepperX.setMaxSpeed(velocitaMotore1);
            }
        }

        if (abs(letturaPot2 - ultimaLetturaPot2) > 50)
        {
            velocitaMotore2 = map(letturaPot2, 0, 4095, VELOCITA_MIN, VELOCITA_MAX);
            ultimaLetturaPot2 = letturaPot2;
            if (currentState == STATE_READY || currentState == STATE_RUNNING)
            {
                stepperY.setMaxSpeed(velocitaMotore2);
            }
        }

        ultimoAggiornamentoPot = millis();
    }
}

// =========== HOMING INTELLIGENTE ===========
void professionalHoming()
{
    if (!homingInCorso) return;

    unsigned long currentTime = millis();

    // Timeout sicurezza
    if (currentTime - homingStartTime > 30000)
    {
        Serial.println("HOMING TIMEOUT");
        emergencyStop();
        return;
    }

    // Homing asse X
    if (!homingXCompletato)
    {
        if (digitalRead(LIMIT_X_MIN) == LOW)
        {
            // Sensore attivato
            stepperX.stop();
            stepperX.setCurrentPosition(0);
            homingXCompletato = true;
            Serial.println("Homing X completato");
            
            // Allontanamento
            stepperX.move(1000);
            stepperX.setSpeed(300);
        }
        else
        {
            // Cerca finecorsa
            stepperX.setSpeed(-300);
            stepperX.runSpeed();
        }
        return;
    }

    // Homing asse Y
    if (!homingYCompletato)
    {
        if (digitalRead(LIMIT_Y_MIN) == LOW)
        {
            // Sensore attivato
            stepperY.stop();
            stepperY.setCurrentPosition(0);
            homingYCompletato = true;
            Serial.println("Homing Y completato");
            
            // Allontanamento
            stepperY.move(1000);
            stepperY.setSpeed(300);
        }
        else
        {
            // Cerca finecorsa
            stepperY.setSpeed(-300);
            stepperY.runSpeed();
        }
        return;
    }

    // Homing completato
    if (homingXCompletato && homingYCompletato)
    {
        // Completa allontanamento
        if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0)
        {
            homingInCorso = false;
            currentState = STATE_READY;
            stepperX.setCurrentPosition(0);
            stepperY.setCurrentPosition(0);
            Serial.println("HOMING COMPLETATO");
        }
    }
}

// =========== CICLO COORDINATO ===========
void cicloCoordinato()
{
    if (!cicloAttivo)
    {
        cicloAttivo = true;
        motore1Completato = false;
        motore2Completato = false;
        tempoInizioCiclo = millis();
        abilitaMotori();
        attivaRele();
        Serial.println("=== INIZIO CICLO COORDINATO ===");

        // ⭐ AVVIA ENTRAMBI INSIEME
        targetPositions[0] = 50000;  // X
        targetPositions[1] = 50000;  // Y
        steppers.moveTo(targetPositions);
        return;
    }

    // ⭐ CONTROLLO FINE CICLO
    if (motore1Completato && digitalRead(LIMIT_X_MIN) == LOW)
    {
        Serial.println("🎯 CICLO COMPLETATO");
        fermaTutto();
        return;
    }

    // ⭐ CONTROLLO: quando X raggiunge MAX
    if (!motore1Completato && digitalRead(LIMIT_X_MAX) == LOW)
    {
        Serial.println("Motore 1 a MAX - ritorno rapido");

        // Ferma Y e spegni relè
        stepperY.stop();
        disattivaRele();
        motore2Completato = true;

        // Ritorno rapido X
        stepperX.setMaxSpeed(velocitaMotore1 * 2);
        stepperX.moveTo(-50000);
        motore1Completato = true;
    }

    // ⭐ CONTROLLO MOTORE 2
    if (!motore2Completato && !motore1Completato)
    {
        controllaMotore2();
    }

    // Controllo timeout
    if (millis() - tempoInizioCiclo > 120000)
    {
        Serial.println("⏰ TIMEOUT CICLO");
        emergencyStop();
    }
}

// =========== GESTIONE MOTORE 2 (OSCILLAZIONE) ===========
void controllaMotore2()
{
    if (motore2Completato || motore1Completato) return;

    static unsigned long ultimoCambio = 0;
    if (millis() - ultimoCambio < 500) return;

    if (digitalRead(LIMIT_Y_MAX) == LOW)
    {
        Serial.println("🔁 M2: MAX → MIN");
        stepperY.moveTo(-50000);
        ultimoCambio = millis();
    }
    else if (digitalRead(LIMIT_Y_MIN) == LOW)
    {
        Serial.println("🔁 M2: MIN → MAX");
        stepperY.moveTo(50000);
        ultimoCambio = millis();
    }
}

// =========== FUNZIONI AUSILIARIE ===========
void fermaTutto()
{
    stepperX.stop();
    stepperY.stop();
    disabilitaMotori();
    disattivaRele();
    cicloAttivo = false;
    motore1Completato = false;
    motore2Completato = false;
    currentState = STATE_READY;
    Serial.println("=== CICLO COMPLETATO ===");
}

void pauseCycle()
{
    Serial.println("⏸️ PAUSA");
    stepperX.stop();
    stepperY.stop();
    stepperX.setMaxSpeed(800);
    stepperX.move(-200);
    disabilitaMotori();
    disattivaRele();
    currentState = STATE_PAUSED;
}

void resumeCycle()
{
    Serial.println("▶️ RIPRESA");
    currentState = STATE_RUNNING;
    abilitaMotori();
    attivaRele();

    if (!motore1Completato)
    {
        stepperX.setMaxSpeed(velocitaMotore1);
        stepperX.moveTo(50000);
    }

    if (!motore2Completato)
    {
        stepperY.setMaxSpeed(velocitaMotore2);
        stepperY.moveTo(50000);
    }
}

// =========== AGGIORNA DISPLAY ===========
void updateDisplay()
{
    display.clearDisplay();

    // Riga 1: Stato sistema
    display.setTextSize(2);
    display.setCursor(0, 0);
    switch (currentState)
    {
    case STATE_OFF: display.print("OFF"); break;
    case STATE_HOMING: display.print("HOMING"); break;
    case STATE_READY: display.print("PRONTO"); break;
    case STATE_RUNNING: display.print("ATTIVO"); break;
    case STATE_PAUSED: display.print("PAUSA"); break;
    case STATE_EMERGENCY: display.print("STOP"); break;
    }

    // Riga 2-3: Stato motori
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("M1:");
    display.print(motore1Completato ? "RITORNO" : "ANDATA");
    display.setCursor(0, 30);
    display.print("M2:AVANTI-INDIETRO");

    // Riga 4: Rele e motori
    display.setCursor(0, 40);
    display.print("RELE:");
    display.print(releAttivo ? "ON " : "OFF");
    display.setCursor(70, 40);
    display.print("MOT:");
    display.print(motoriAbilitati ? "ON" : "OFF");

    // Riga 5: Velocità
    display.setCursor(0, 55);
    display.print("V1:");
    display.print(velocitaMotore1);
    display.print("sps");
    display.setCursor(65, 55);
    display.print("V2:");
    display.print(velocitaMotore2);
    display.print("sps");

    // Riga 6: Finecorsa X
    display.setCursor(0, 70);
    display.print("X_MIN:");
    display.print(digitalRead(LIMIT_X_MIN) == LOW ? "ATT" : "NO");
    display.setCursor(70, 70);
    display.print("X_MAX:");
    display.print(digitalRead(LIMIT_X_MAX) == LOW ? "ATT" : "NO");

    // Riga 7: Finecorsa Y
    display.setCursor(0, 85);
    display.print("Y_MIN:");
    display.print(digitalRead(LIMIT_Y_MIN) == LOW ? "ATT" : "NO");
    display.setCursor(70, 85);
    display.print("Y_MAX:");
    display.print(digitalRead(LIMIT_Y_MAX) == LOW ? "ATT" : "NO");

    // Riga 8: Comandi
    display.setCursor(0, 100);
    display.print("START:");
    switch (currentState)
    {
    case STATE_OFF: display.print("Homing"); break;
    case STATE_READY: display.print("Avvia"); break;
    case STATE_PAUSED: display.print("Riprendi"); break;
    case STATE_EMERGENCY: display.print("Reset"); break;
    default: display.print("---"); break;
    }

    // Riga 9: Stato ciclo
    display.setCursor(0, 115);
    display.print("CICLO:");
    display.print(cicloAttivo ? "ATTIVO" : "FERMO");

    display.display();
}

// =========== DEBUG RELE ===========
void debugRele()
{
    static bool ultimoStatoRele = false;
    if (releAttivo != ultimoStatoRele)
    {
        Serial.printf("🔌 RELE: %s -> %s\n", ultimoStatoRele ? "ON" : "OFF", releAttivo ? "ON" : "OFF");
        ultimoStatoRele = releAttivo;
    }
}

// =========== LOOP PRINCIPALE ===========
void loop()
{
    // ⭐ UNICA CHIAMATA PER ENTRAMBI GLI STEPPER
    steppers.run();

    checkButtons();
    aggiornaPotenziometri();
    debugRele();

    switch (currentState)
    {
    case STATE_HOMING:
        professionalHoming();
        break;
    case STATE_RUNNING:
        cicloCoordinato();
        break;
    case STATE_PAUSED:
        // Durante pausa permette a X di completare la retrocessione
        stepperX.run();
        break;
    default:
        break;
    }

    updateDisplay();
    delay(50);
}