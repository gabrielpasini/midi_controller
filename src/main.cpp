#define DEBUG 1

#include "MIDIUSB.h"
#include <ResponsiveAnalogRead.h>

const int N_BUTTONS = 2;                              // número total de botões
const int BUTTON_ARDUINO_PIN[N_BUTTONS] = { 6, 7 };  // pinos de cada botão conectado diretamente ao Arduino

int buttonCState[N_BUTTONS] = {};  // armazena o valor atual do botão
int buttonPState[N_BUTTONS] = {};  // armazena o valor anterior do botão

//#define pin13 1 // descomente se estiver usando o pino 13 (pino com led), ou comente a linha se não estiver usando
byte pin13index = 12;  // coloque o índice do pino 13 do array de pinos do botãoPin[] se estiver usando, se não, comente

unsigned long lastDebounceTime[N_BUTTONS] = { 0 };  // a última vez que o pino de saída foi alternado
unsigned long debounceDelay = 50;                   // o tempo de debounce; aumente se a saída oscilar

const int N_POTS = 1;                         // número total de potenciômetros (slide e rotativo)
const int POT_ARDUINO_PIN[N_POTS] = { A2 };  // pinos de cada potenciômetro conectado diretamente ao Arduino

int potCState[N_POTS] = { 0 };  // estado atual do potenciômetro
int potPState[N_POTS] = { 0 };  // estado anterior do potenciômetro
int potVar = 0;                 // diferença entre o estado atual e anterior do potenciômetro

int midiCState[N_POTS] = { 0 };  // estado atual do valor midi
int midiPState[N_POTS] = { 0 };  // estado anterior do valor midi

const int TIMEOUT = 300;              // Tempo em que o potenciômetro será lido após exceder o varThreshold
const int varThreshold = 20;          // Limiar para a variação do sinal do potenciômetro
boolean potMoving = true;             // Se o potenciômetro está se movendo
unsigned long PTime[N_POTS] = { 0 };  // Tempo previamente armazenado
unsigned long timer[N_POTS] = { 0 };  // Armazena o tempo passado desde que o temporizador foi redefinido

int reading = 0;
float snapMultiplier = 0.01;                      // (0.0 - 1.0) - Aumente para uma leitura mais rápida, mas menos suave
ResponsiveAnalogRead responsivePot[N_POTS] = {};  // cria um array para os potenciômetros responsivos. Ele é preenchido no Setup.

int potMin = 10;
int potMax = 1023;

byte midiCh = 0;  // Canal MIDI a ser usado - comece com 1 para a biblioteca MIDI.h ou 0 para a biblioteca MIDIUSB
byte note = 36;   // Nota mais baixa a ser usada
byte cc = 1;      // Menor CC MIDI a ser usado

void setup() {
  // 31250 para classe de compatibilidade MIDI | 115200 para Hairless MIDI
  Serial.begin(115200);  //

#ifdef DEBUG
  Serial.println("Modo de depuração");
  Serial.println();
#endif

  for (int i = 0; i < N_BUTTONS; i++) {
    pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);
  }

#ifdef pin13  // inicializa o pino 13 como entrada
  pinMode(BUTTON_ARDUINO_PIN[pin13index], INPUT);
#endif

  for (int i = 0; i < N_POTS; i++) {
    responsivePot[i] = ResponsiveAnalogRead(0, true, snapMultiplier);
    responsivePot[i].setAnalogResolution(1023);  // define a resolução
  }
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = { 0x09, 0x90 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = { 0x08, 0x80 | channel, pitch, velocity };
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = { 0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
}

void buttons() {

  for (int i = 0; i < N_BUTTONS; i++) {
    buttonCState[i] = digitalRead(BUTTON_ARDUINO_PIN[i]);  // lê os pinos do arduino

#ifdef pin13
    if (i == pin13index) {
      buttonCState[i] = !buttonCState[i];  // inverte o pino 13 porque ele possui um resistor pull-down em vez de um pull-up
    }
#endif

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (buttonPState[i] != buttonCState[i]) {
        lastDebounceTime[i] = millis();
        if (buttonCState[i] == LOW) {
          noteOn(midiCh, note + i, 127);  // canal, nota, velocidade
          MidiUSB.flush();

#ifdef DEBUG
          Serial.print(i);
          Serial.println(": botão ligado");
#endif

        } else {
          noteOn(midiCh, note + i, 0);  // canal, nota, velocidade
          MidiUSB.flush();

#ifdef DEBUG
          Serial.print(i);
          Serial.println(": botão desligado");
#endif

        }
        buttonPState[i] = buttonCState[i];
      }
    }
  }
}

void potentiometers() {
  for (int i = 0; i < N_POTS; i++) {  // Percorre todos os potenciômetros

    reading = analogRead(POT_ARDUINO_PIN[i]);
    responsivePot[i].update(reading);
    potCState[i] = responsivePot[i].getValue();
    potCState[i] = analogRead(POT_ARDUINO_PIN[i]);  // lê os pinos do arduino
    midiCState[i] = map(potCState[i], potMin, potMax, 0, 127);  // Mapeia a leitura de potCState para um valor utilizável em midi
    //midiCState[i] = map(potCState[i], 0, 4096, 0, 127);  // Mapeia a leitura de potCState para um valor utilizável em midi - usar para ESP32
    if (midiCState[i] < 0) {
      midiCState[i] = 0;
    }
    if (midiCState[i] > 127) {
      midiCState[i] = 0;
    }
    potVar = abs(potCState[i] - potPState[i]);  // Calcula o valor absoluto entre a diferença entre o estado atual e anterior do potenciômetro
    if (potVar > varThreshold) {  // Abre o portão se a variação do potenciômetro for maior que o limiar
      PTime[i] = millis();        // Armazena o tempo anterior
    }
    timer[i] = millis() - PTime[i];  // Redefine o temporizador 11000 - 11000 = 0ms
    if (timer[i] < TIMEOUT) {       // Se o temporizador for menor que o tempo máximo permitido, significa que o potenciômetro ainda está se movendo
      potMoving = true;
    } else {
      potMoving = false;
    }
    if (potMoving == true) {  // Se o potenciômetro ainda estiver se movendo, envie a mudança de controle
      if (midiPState[i] != midiCState[i]) {
        controlChange(midiCh, cc + i, midiCState[i]);  //  (canal, número do CC, valor do CC)
        MidiUSB.flush();

#ifdef DEBUG
        Serial.print("Potenciômetro: ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(midiCState[i]);
#endif

        potPState[i] = potCState[i];  // Armazena a leitura atual do potenciômetro para comparar com a próxima
        midiPState[i] = midiCState[i];
      }
    }
  }
}

void loop() {
  buttons();
  potentiometers();
}