/*
 *  The Teensy code for resampling async I2S2 Input is credited to Alex6679
 *  https://github.com/alex6679/ESP32_I2S_Teensy4
 *
 * Matrix Mixer from https://github.com/palmerr23/TeensyAudioMatrixMixer
*/

#include <Audio.h>
#include <SPI.h>
#include <SSD1306Wire.h>    // OLED library
#include <Encoder.h>        // Rotary encoder library
#include <ArduinoJson.h>    // JSON library
#include <SD.h>             // SD card library
#include <Bounce.h>         // Débounce des Switchs
#include <Wire.h>
#include <SerialFlash.h>
#include "mixerMatrix.h"

// I2S Async Input (Bluetooth Player)
#include "async_input.h"
#include "input_i2s2_16bit.h"
#include "output_i2s2_16bit.h"
#include "plotter.h"

//*********************************************************************************
//*****************************Déclaration des constantes *************************
//*********************************************************************************

/* Pins utilisées :
0 -- Rx1
1 -- Tx1
2 -Esp32-Mini RESET- (Pin RST)
3 -Esp32-Mini asyncI2S LRCLK2- (Pin 16)
4 -Esp32-Mini asyncI2S BCLK2- (Pin 17)
5 -Esp32-Mini asyncI2S IN2- (Pin 21)
6 Encodeur Rotatif 1B
7 I2S OUT1A (AudioBoard)
8 I2S IN1 (AudioBoard)
9 Encodeur Rotatif 1A
10 AudioSD CS
11 AudioSD MOSI
12 AudioSD MISO
13 AudioSD SCK + Local LED
14 Switch Encodeur 1
15 Switch Encodeur 2 -ou Pot Analog-
16 Encodeur 2A
17 Encodeur 2B
18 SDA0 (AudioBoard // OLED)
19 SCL0 (AudioBoard // OLED)
20 I2S LRCLCK (AudioBoard)
21 I2S BCLK1 (AudioBoard)
22 
23 I2S MCLK1 (AudioBoard)
24 Switch 3
25 Green Led 1
26 Switch 4
27 Green Led 2
28 Switch 5
29 Green Led 3
*/

// Pin Teensy > Reset de l'Esp32 (Pb volume bluetooth)
const int resetPin = 2;

// 3 Switchs de fonction :
Bounce pushButton3 = Bounce(24, 10); // 10 ms debounce
Bounce pushButton4 = Bounce(26, 10);
Bounce pushButton5 = Bounce(28, 10);
// Etat des boutons pour faire du Latch :
bool button3State = false;
bool button4State = false;
bool button5State = LOW;

// 3 LED Vertes de fonction :
const int ledGreen1 = 25;
const int ledGreen2 = 27;
const int ledGreen3 = 29;

// BPM & LED 3
unsigned long lastBeatTime = 0;
bool led3State = false;
const long interval = 100;

// Pins i2c configuration
const int OLED_SDA = 18; 
const int OLED_SCL = 19;

// Pïns Encodeur rotatif 1 + Switch 1
const int ENCODER_PIN_1A = 9;
const int ENCODER_PIN_1B = 6;
const int ENCODER_SWITCH_1 = 14;
Bounce pushButton = Bounce(ENCODER_SWITCH_1, 10);

// Pïns Encodeur rotatif 2 + Switch 2
const int ENCODER_PIN_2A = 16;
const int ENCODER_PIN_2B = 17;
const int ENCODER_SWITCH_2 = 15; 
Bounce pushButton2 = Bounce(ENCODER_SWITCH_2, 10);

/*
// Broche du potentiomètre Analog pour volume casque
// const int potPin = 15; // A1 sur Teensy 4.0
// const float exponent = 2.0;
// elapsedMillis volmsec=0;
*/

// Pins Carte SD
const int SD_CS_PIN = 10;

// Screen and encoder setup
SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
Encoder encoder(ENCODER_PIN_1A, ENCODER_PIN_1B); // Encodeur Gestion Menu ...
Encoder encoder2(ENCODER_PIN_2A, ENCODER_PIN_2B); // Encodeur Gros Volume ...
long lastEncoder1Pos = 0;
long lastEncoder2Pos = -999;

// Variables Audio
float volume = 0.50;
float roomSize = 0.50;
float damping = 0.50;
float cleanDelay = 1000;
float cleanDelay2 = 500;
float toneFrequency = 1000;

// Variables Audio Matrix 8x8
float i2sToi2s = 0;
float i2sToUsb = 0;
float i2sToRvb = 0;
float i2sToDelay = 0;
float btToi2s = 0;
float btToUsb = 0;
float btToRvb = 0;
float btToDelay = 0;
float usbToi2s = 0;
float usbToUsb = 0;
float usbToRvb = 0;
float usbToDelay = 0;
float rvbToi2s = 0;
float rvbToUsb = 0;
float rvbToRvb = 0;
float rvbToDelay = 0;
float delay1Toi2s = 0;
float delay1ToUsb = 0;
float delay1ToRvb = 0;
float delay1ToDelay = 0;
float delay2Toi2s = 0;
float delay2ToUsb = 0;
float delay2ToRvb = 0;
float delay2ToDelay = 0;
float toneToi2s = 0;
float toneToUsb = 0;
float toneToRvb = 0;
float toneToDelay = 0;

//*********************************************************************************
//*************************       TAP TEMPO & DELAY   *****************************
//*********************************************************************************
unsigned long lastTapTime = 0;
unsigned long prevTapTime = 0;
float tapTempoMs = 500.0;
float tapTempoBpm = 120;
bool tempoValid = false;

// Subdivisions disponibles
const float subdivisions[] = {1.0, 0.5, 1.5, 0.333};
const char* subdivisionNames[] = {"1/1", "1/2", "1.5", "1/3"};
int currentSubdivision = 0;

//*********************************************************************************
//**************************   Fonctions utilisées   ******************************
//*********************************************************************************
void drawVolumePage();
void drawFxPage();
void drawMenu();
void onButtonPress();
void resetEsp();
void lightLed1();
void lightLed2();
void bpmLed3();

void executeAmixerSetup();
void executeVolume();
void executeMatrix();
void executeReverb();
void executeDelay();
void executeTone();
void modifyVariable(float &variable, long encoderPos, float step, float minVal = 0.0, float maxVal = 1.0);

void updateTapTempo();
void applyDelayTime();
void cycleSubdivision();

void saveConfig(const char* fileNameToSave);
void loadConfig(const char* fileNameToLoad);
void saveLastConfig(const char* fileNameToSaveLast);
const char* loadLastConfig();
const char* selectedConfig;
const char* lastConfigToUse;
// Variables pour la Sauvegarde automatique dans /temp.json :
unsigned long previousMillis2 = 0;
const long interval2 = 600000;     // Délai avant chaque sauvegarde automatique (en ms)

//*********************************************************************************
//*****************  Choix du mode d'affichage Principal  *************************
//*********************************************************************************
bool mainMode = false;
void modeVolumeSelect();
void modeFxSelect();
//*********************************************************************************
//*************   Structure de l'affichage Principal en mode Fx   *****************
//*********************************************************************************
// Structure
enum ParamType { FLOAT_0_1, FLOAT_0_10000 };

struct Param {
  const char* label;
  float* value;
  ParamType type;
  void (*onChange)();
};

struct Page {
  const char* name;
  Param* params[6]; // pointeurs vers Param ou nullptr
};

// Paramètres des pages réglages Fx
Param paramReverb1 = { "RoomSize", &roomSize, FLOAT_0_1, executeReverb };
Param paramReverb2 = { "Damping", &damping, FLOAT_0_1, executeReverb };
Param paramReverb3 = { "i2sToRvb", &i2sToRvb, FLOAT_0_1, executeReverb };
Param paramReverb4 = { "btToRvb", &btToRvb, FLOAT_0_1, executeReverb };
Param paramReverb5 = { "usbToRvb", &usbToRvb, FLOAT_0_1, executeReverb };
Param paramReverb6 = { "rvbToi2s", &rvbToi2s, FLOAT_0_1, executeReverb };
Param paramReverb11 = { "rvbToUsb", &rvbToUsb, FLOAT_0_1, executeReverb };
Param paramReverb12 = { "rvbToRvb", &rvbToRvb, FLOAT_0_1, executeReverb };
Param paramReverb13 = { "rvbToDelay", &rvbToDelay, FLOAT_0_1, executeReverb };

Param paramDelay1 = { "Delay time", &cleanDelay, FLOAT_0_10000, executeDelay };
Param paramDelay2 = { "i2sToDelay", &i2sToDelay, FLOAT_0_1, executeDelay };
Param paramDelay3 = { "btToDelay", &btToDelay, FLOAT_0_1, executeDelay };
Param paramDelay4 = { "usbToDelay", &usbToDelay, FLOAT_0_1, executeDelay };
Param paramDelay11 = { "delay1Toi2s", &delay1Toi2s, FLOAT_0_1, executeDelay };
Param paramDelay12 = { "delay1ToUsb", &delay1ToUsb, FLOAT_0_1, executeDelay };
Param paramDelay13 = { "delay1ToRvb", &delay1ToRvb, FLOAT_0_1, executeDelay };
Param paramDelay14 = { "delay1ToDelay", &delay1ToDelay, FLOAT_0_1, executeDelay };
Param paramDelay21 = { "Delay 2 time", &cleanDelay2, FLOAT_0_1, executeDelay };
Param paramDelay22 = { "delay2Toi2s", &delay2Toi2s, FLOAT_0_1, executeDelay };
Param paramDelay23 = { "delay2ToUsb", &delay2ToUsb, FLOAT_0_1, executeDelay };
Param paramDelay24 = { "delay2ToRvb", &delay2ToRvb, FLOAT_0_1, executeDelay };
Param paramDelay25 = { "delay2ToDelay", &delay2ToDelay, FLOAT_0_1, executeDelay };

Param paramTone1 = { "Frequence", &toneFrequency, FLOAT_0_10000, executeTone };
Param paramTone2 = { "toneToi2s", &toneToi2s, FLOAT_0_1, executeTone };
Param paramTone3 = { "toneToUsb", &toneToUsb, FLOAT_0_1, executeTone };
Param paramTone4 = { "toneToRvb", &toneToRvb, FLOAT_0_1, executeTone };
Param paramTone5 = { "toneToDelay", &toneToDelay, FLOAT_0_1, executeTone };

Param paramMatrix11 = { "i2sToi2s", &i2sToi2s, FLOAT_0_1, executeMatrix };
Param paramMatrix12 = { "i2sToUsb", &i2sToUsb, FLOAT_0_1, executeMatrix };
Param paramMatrix13 = { "btToi2s", &btToi2s, FLOAT_0_1, executeMatrix };
Param paramMatrix14 = { "btToUsb", &btToUsb, FLOAT_0_1, executeMatrix };
Param paramMatrix15 = { "usbToi2s", &usbToi2s, FLOAT_0_1, executeMatrix };
Param paramMatrix16 = { "usbToUsb", &usbToUsb, FLOAT_0_1, executeMatrix };

// Pages réglages Fx
Page pages[] = {
  { "REVERB 1", { &paramReverb1, &paramReverb2, &paramReverb3, &paramReverb4, &paramReverb5, &paramReverb6 } },
  { "REVERB 2", { &paramReverb11, &paramReverb12, &paramReverb13, nullptr, nullptr, nullptr } },
  { "DELAY 1", { &paramDelay1, &paramDelay21, &paramDelay2, &paramDelay3, &paramDelay4, nullptr } },
  { "DELAY 2", { &paramDelay1, &paramDelay21, &paramDelay11, &paramDelay12, &paramDelay13, &paramDelay14 } },
  { "DELAY 3", { &paramDelay1, &paramDelay21, &paramDelay22, &paramDelay23, &paramDelay24, &paramDelay25 } },
  { "TONE", { &paramTone1, &paramTone2, &paramTone3, &paramTone4, &paramTone5, nullptr } },
  { "Fx 4", { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr } },
  { "MATRIX", { &paramMatrix11, &paramMatrix12, &paramMatrix13, &paramMatrix14, &paramMatrix15, &paramMatrix16 } }
};
// Navigation
const int numPages = sizeof(pages) / sizeof(Page);
int currentPage = 0;
int selectedParam = 0;
bool editing = false;

//*********************************************************************************
//*****************************   Structure du menu   *****************************
//*********************************************************************************
// --- Structure de menu ---
enum MenuItemType { ACTION, SUBMENU, FLOAT_EDIT, FLOAT_LARGE_EDIT};

struct MenuItem {
  String label;            // A afficher
  MenuItemType type;       // Declenchements
  float* floatVar;         // Pour FLOAT_EDIT
  MenuItem* subMenu;       // Pour SUBMENU
  int numSubItems;         // Nb entrée sur la page à afficher
  void (*actionFunc)();    // Pour déclencher une fonction
  void (*actionFuncWithArg)(const char*);  // Fonction avec argument
  const char* argStr = nullptr;            // L'argument à passer
};

// --- Sous-menus ---
MenuItem submenu1[] = { // Reverb
  {"Send Analog", FLOAT_EDIT, &i2sToRvb, nullptr, 0},
  {"Send Bluetooth", FLOAT_EDIT, &btToRvb, nullptr, 0},
  {"Send Usb", FLOAT_EDIT, &usbToRvb, nullptr, 0},
  {"Send Rvb", FLOAT_EDIT, &rvbToRvb, nullptr, 0},
  {"Send Delay1", FLOAT_EDIT, &delay1ToRvb, nullptr, 0},
  {"Send Delay2", FLOAT_EDIT, &delay2ToRvb, nullptr, 0},
  {"Roomsize", FLOAT_EDIT, &roomSize, nullptr, 0},
  {"Damping", FLOAT_EDIT, &damping, nullptr, 0},
  {"Master RVB>Analog", FLOAT_EDIT, &rvbToi2s, nullptr, 0},
  {"Master RVB>USB", FLOAT_EDIT, &rvbToUsb, nullptr, 0},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu2[] = { // Delay
  {"Send Analog", FLOAT_EDIT, &i2sToDelay, nullptr, 0},
  {"Send Bluetooth", FLOAT_EDIT, &btToDelay, nullptr, 0},
  {"Send Usb", FLOAT_EDIT, &usbToDelay, nullptr, 0},
  {"Send Rvb", FLOAT_EDIT, &rvbToDelay, nullptr, 0},
  {"Send Delay1", FLOAT_EDIT, &delay1ToDelay, nullptr, 0},
  {"Send Delay2", FLOAT_EDIT, &delay2ToDelay, nullptr, 0},
  {"Delay Time 1", FLOAT_LARGE_EDIT, &cleanDelay, nullptr, 0},
  {"Subdivisions", ACTION, nullptr, nullptr, 0, cycleSubdivision},
  {"Tempo Ms", FLOAT_LARGE_EDIT, &tapTempoMs, nullptr, 0},
  {"Tempo BPM", FLOAT_LARGE_EDIT, &tapTempoBpm, nullptr, 0},
  {"Delay Time 2", FLOAT_LARGE_EDIT, &cleanDelay2, nullptr, 0},
  {"Master Delay1>Analog", FLOAT_EDIT, &delay1Toi2s, nullptr, 0},
  {"Master Delay1>USB", FLOAT_EDIT, &delay1ToUsb, nullptr, 0},
  {"Master Delay2>Analog", FLOAT_EDIT, &delay2Toi2s, nullptr, 0},
  {"Master Delay2>USB", FLOAT_EDIT, &delay2ToUsb, nullptr, 0},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu3[] = { // Tone
  {"Send to Analog", FLOAT_EDIT, &toneToi2s, nullptr, 0},
  {"Send to Usb", FLOAT_EDIT, &toneToUsb, nullptr, 0},
  {"Send to Rvb", FLOAT_EDIT, &toneToRvb, nullptr, 0},
  {"Send to Delay", FLOAT_EDIT, &toneToDelay, nullptr, 0},
  {"Frequence", FLOAT_LARGE_EDIT, &toneFrequency, nullptr, 0},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu4[] = { // Leds
  {"Led1", ACTION, nullptr, nullptr, 0, lightLed1},
  {"Led2", ACTION, nullptr, nullptr, 0, lightLed2},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu6[] = { // Snapshots Load
  {"Load User1", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/user1.json"},
  {"Load User2", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/user2.json"},
  {"Load User3", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/user3.json"},
  {"Load Volume", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/volume.json"},
  {"Load Bluetooth", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/bluetooth.json"},
  {"Load Margot", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/margot.json"},
  {"Load MultiFx", ACTION, nullptr, nullptr, 0, nullptr, loadConfig, "/multiFx.json"},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu7[] = { // Snapshots Save
  {"Save User1", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/user1.json"},
  {"Save User2", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/user2.json"},
  {"Save User3", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/user3.json"},
  {"Save Volume", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/volume.json"},
  {"Save Bluetooth", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/bluetooth.json"},
  {"Save Margot", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/margot.json"},
  {"Save MultiFx", ACTION, nullptr, nullptr, 0, nullptr, saveConfig, "/multiFx.json"},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
MenuItem submenu5[] = { // Snapshots
  {"Load Snapshot", SUBMENU, nullptr, submenu6, 8},
  {"Save Snapshot", SUBMENU, nullptr, submenu7, 8},
  {"Retour", ACTION, nullptr, nullptr, 0}
};
// --- Menu principal ---
MenuItem mainMenu[] = {
  {"Volume", FLOAT_EDIT, &volume, nullptr, 0},
  {"Niveau Analog", FLOAT_EDIT, &i2sToi2s, nullptr, 0},
  {"Niveau BT", FLOAT_EDIT, &btToi2s, nullptr, 0},
  {"Niveau Usb", FLOAT_EDIT, &usbToi2s, nullptr, 0},
  {"Reverb", SUBMENU, nullptr, submenu1, 11},
  {"Delay", SUBMENU, nullptr, submenu2, 16},
  {"Tone", SUBMENU, nullptr, submenu3, 6},
  {"Led", SUBMENU, nullptr, submenu4, 3},
  {"Snapshots", SUBMENU, nullptr, submenu5, 3},
  {"Mode Volume", ACTION, nullptr, nullptr, 0, modeVolumeSelect},
  {"Mode Fx", ACTION, nullptr, nullptr, 0, modeFxSelect},
  {"Reset ESPmini", ACTION, nullptr, nullptr, 0, resetEsp},
  {"Quitter", ACTION, nullptr, nullptr, 0}
};

// --- Navigation ---
MenuItem* currentMenu = mainMenu;
int currentMenuLength = 13;
int selectedIndex = 0;
int menuOffset = 0;
bool inMenu = false;
bool editingFloat = false;
bool editingFloatLarge = false;

// --- Stack pour naviguer entre menus ---
const int maxMenuDepth = 5;
MenuItem* menuStack[maxMenuDepth];
int menuLengths[maxMenuDepth];
int stackIndex = 0;

// --- Affichage ---
const int visibleLines = 6;
const int lineHeight = 10;

// Image de Volume Ecran principal
const unsigned char volumeBitmap [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x08, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x1c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x3c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x07, 0x3c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x07, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xc7, 0x70, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xe7, 0x71, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xe7, 0xe1, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xc7, 0xe1, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xc7, 0xe3, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xe3, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xc3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xc3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xc3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xc3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xc3, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xe3, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x87, 0xe3, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xc7, 0xe3, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xc7, 0xe1, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xe7, 0xf1, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xe7, 0x70, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x07, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x07, 0x38, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x3c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x1c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x1c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x07, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//*********************************************************************************
//****************************      Routing Audio       ***************************
//*********************************************************************************

// i2sSlaveInput parameters (i2s n°2 du Teensy avec SRC)
Plotter plotter(8);
 bool dither = false;
 bool noiseshaping = false;
 float attenuation = 100;
 int32_t minHalfFilterLength=80;
 int32_t maxHalfFilterLength=1;
 AsyncAudioInput<AsyncAudioInputI2S2_16bitslave> i2sSlaveInput(dither, noiseshaping, attenuation, minHalfFilterLength, maxHalfFilterLength);
//AsyncAudioInput<AsyncAudioInputI2S2_16bitslave> i2sSlaveInput;

// GUItool: begin automatically generated code
AudioEffectFreeverbStereo freeverbs1;     //xy=381.2222366333008,400.7777490615845
AudioEffectDelay         delay1;         //xy=143.3333625793457,628.8888778686523   
AudioSynthWaveformSine   tone1;          //xy=343.3333396911621,617.7777614593506   

AudioInputI2S            i2sInput;       //xy=66.4444465637207,192.55555725097656
AudioInputUSB            usbInput;       //xy=388.4444465637207,355.55555725097656

// Matrix Mixer 16x16 
AudioMixerMatrix Amixer(16, 16);

AudioAmplifier           ampRight;       //xy=845.1110610961914,207.88889026641846
AudioAmplifier           ampLeft;        //xy=846.7778015136719,170.77778148651123
AudioOutputI2S           i2sOutput;      //xy=990.7777557373047,187.55556678771973
AudioOutputUSB           usbOutput;      //xy=257.4444465637207,280.55555725097656

AudioConnection          patchCord1(i2sInput, 0, Amixer, 0);
AudioConnection          patchCord2(i2sInput, 1, Amixer, 1);
AudioConnection          patchCord3(i2sSlaveInput, 0, Amixer, 2);
AudioConnection          patchCord4(i2sSlaveInput, 1, Amixer, 3);
AudioConnection          patchCord5(usbInput, 0, Amixer, 4);
AudioConnection          patchCord6(usbInput, 1, Amixer, 5);
AudioConnection          patchCord7(freeverbs1, 0, Amixer, 6);
AudioConnection          patchCord8(freeverbs1, 1, Amixer, 7);
AudioConnection          patchCord9(delay1, 0, Amixer, 8);
AudioConnection          patchCord10(delay1, 1, Amixer, 9);
AudioConnection          patchCord11(tone1, 0, Amixer, 10);

AudioConnection          patchCord12(Amixer, 0, ampLeft, 0);
AudioConnection          patchCord13(Amixer, 1, ampRight, 0);
AudioConnection          patchCord14(Amixer, 2, usbOutput, 0);
AudioConnection          patchCord15(Amixer, 3, usbOutput, 1);
AudioConnection          patchCord16(Amixer, 4, freeverbs1, 0);
AudioConnection          patchCord17(Amixer, 5, delay1, 0);

AudioConnection          patchCord18(ampLeft, 0, i2sOutput, 0);
AudioConnection          patchCord19(ampRight, 0, i2sOutput, 1);



AudioControlSGTL5000     audioShield;    //xy=617.4444465637207,94.55555725097656
// GUItool: end automatically generated code
/*
Réglages AudioMatrix :
matrix.gain(unsigned int inChannel, unsigned int outChannel, float gain);

Réglages AudioMixer4 :
mixerLeft.gain(unsigned int channel, float gain);

Réglages freeverbs1 :
freeverbs1.roomsize(float n);
freeverbs1.damping(float n);

Réglages Délay :
delay1.delay(uint8_t channel, float milliseconds);
delay1.disable(bool n);

Réglages AudioSynthWaveformSine :
sine1.frequency(float freq);
sine1.amplitude(float n);
sine1.phase(float angle);
*/

//*********************************************************************************
//*****************************       SETUP      **********************************
//*********************************************************************************
void setup() {
  // Allocation de la mémoire pour le processing Audio :
  AudioMemory(1024);
  // Matrix à ZERO :
  executeAmixerSetup();
  // Rappel des Parametres Audio Statiques
  executeAllStaticVariables();
  // Niveau de sortie analogique à ZERO ("volume")
  ampLeft.gain(0.0);
  ampRight.gain(0.0);

  // Démarrage du port Série USB
  Serial.begin(9600);
  Serial.println("Démarrage du Teensy...");

  // Initialisation de l'afficheur OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(30, 20, "Welcome in ...");
  display.setFont(ArialMT_Plain_16);
  display.drawString(35, 40, "Teensy !");
  display.setFont(ArialMT_Plain_10);
  display.display();
  delay(1000);

  // Config du Reset de l'esp32
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH); // Pas de reset de l'esp32mini pour l'instant
  // Config du switch de l'encodeur rotatif 1 et 2
  pinMode(ENCODER_SWITCH_1, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH_2, INPUT_PULLUP);
  // Config des 3 switchs de fonction
  pinMode(24, INPUT_PULLDOWN);
  pinMode(26, INPUT_PULLDOWN);
  pinMode(28, INPUT_PULLDOWN);
  // Config des Leds Verte
  pinMode(ledGreen1, OUTPUT);
  pinMode(ledGreen2, OUTPUT);
  pinMode(ledGreen3, OUTPUT);
  // Initialisation des Led Eteintes
  digitalWrite(ledGreen1, HIGH);
  digitalWrite(ledGreen2, HIGH);
  digitalWrite(ledGreen3, HIGH);

  // Initialisation du shield audio
  if (!audioShield.enable()) 
  {
    Serial.println("Erreur Audio Board...");
    digitalWrite(ledGreen1, LOW); // 3 LED Verte allumées si erreur du Shield Audio
    digitalWrite(ledGreen2, LOW);
    digitalWrite(ledGreen3, LOW);
    display.clear();
    display.drawString(0, 0, "Erreur AudioBoard");
    display.display();
  } else {
    Serial.println("AudioBoard démarré !");
    audioShield.inputSelect(0);
    audioShield.inputLevel(0); 
    audioShield.unmuteLineout();
    audioShield.lineOutLevel(13); // 13 > 31
    audioShield.volume(0.5); // Volume initial du Headphone
  }

  // Initialisation de la carte SD
  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("Erreur Carte SD");
    display.drawString(0, 20, "Erreur SD");
    display.display();
  }

  // Verification de la derniere config utilisée
  selectedConfig = loadLastConfig();
  Serial.println("loadLastConfig() :");
  Serial.println(selectedConfig);
  // Chargement de la config
  loadConfig(selectedConfig);
  delay(1000);

  // Reset de l'esp32 pour régler le probleme de variation de volume du bluetooth :
  resetEsp();

  // Fin du démarrage
  Serial.println("Séquence démarrage du Teensy terminée");
}
//*********************************************************************************
//******************************       LOOP      **********************************
//*********************************************************************************
void loop() {
  pushButton.update();
  pushButton2.update();
  pushButton3.update();
  pushButton4.update();
  pushButton5.update();
  bpmLed3();
  autoSave();
//************************ Gestion Menus et affichage **************************************************
  if (pushButton.fallingEdge()) {
    onButtonPress();
  }
  if (inMenu) {
    if (editingFloat) {
      long encoder1Pos = encoder.read() / 4;
      if (encoder1Pos != lastEncoder1Pos) {
        int incrementEnc1 = (encoder1Pos > lastEncoder1Pos) ? 1 : -1;
        float val = constrain(*currentMenu[selectedIndex].floatVar + incrementEnc1 * 0.02, 0.0, 1.0);
        *currentMenu[selectedIndex].floatVar = val;
        executeAllVariables();
        lastEncoder1Pos = encoder1Pos;
      }
      // ToDO : RECUPERER LA VARIABLE à MODIFIER POUR APPLIQUER seulement la bonne...???
      // Avec *currentMenu[selectedIndex].xxxxxx .....???
    } else if (editingFloatLarge) {
      long encoder1Pos = encoder.read() / 4;
      if (encoder1Pos != lastEncoder1Pos) {
        int incrementEnc1 = (encoder1Pos > lastEncoder1Pos) ? 1 : -1;
        float val = constrain(*currentMenu[selectedIndex].floatVar + incrementEnc1 * 0.02, 0.0, 1.0);
        *currentMenu[selectedIndex].floatVar = val;
        executeAllVariables();
        lastEncoder1Pos = encoder1Pos;
      }
    } else {
      int rawPos = encoder.read() / 4;
      if (rawPos < 0) {
        rawPos = 0;
        encoder.write(0);
      }
      if (rawPos >= currentMenuLength) {
        rawPos = currentMenuLength - 1;
        encoder.write(rawPos * 4);
      }
      selectedIndex = rawPos;
    }
    drawMenu();
  } else if (!inMenu) {       // Affichage Hors Menu
    if (mainMode == true) {   // Mode Fx
      drawFxPage();
      // Encoder 1 : changer de page
      int pageDelta = encoder.read() / 4; // valeur relative
      if (pageDelta != 0 && !editing) {
        currentPage = (currentPage + pageDelta + numPages) % numPages;
        encoder.write(0);
        selectedParam = 0;
      }
      // Encoder 2 : changer de paramètre (si pas en édition)
      int paramDelta = encoder2.read() / 4;
      if (paramDelta != 0 && !editing) {
        int maxParam = countNonNullParams(pages[currentPage]);
        selectedParam = (selectedParam + paramDelta + maxParam) % maxParam;
        encoder2.write(0);
      }
      if (pushButton2.fallingEdge()) {
        editing = !editing;
        encoder2.write(0);
      }
      // Modifier valeur si en édition
      if (editing) {
        Param* p = pages[currentPage].params[selectedParam];
        if (p != nullptr) {
          int delta = encoder2.read();
          if (delta != 0) {
            float step = (p->type == FLOAT_0_1) ? 0.01 : 100;
            float minVal = (p->type == FLOAT_0_1) ? 0.0 : 0.0;
            float maxVal = (p->type == FLOAT_0_1) ? 1.0 : 2000.0;
            *(p->value) = constrain(*(p->value) + delta * step, minVal, maxVal);
            encoder2.write(0);
            if (p->onChange) p->onChange();
          }
        }
      }
    } else if (mainMode == false) {   // Mode Volume
        drawVolumePage();
        // Modification du volume Encodeur 2
        long encoder2Pos = encoder2.read() / 4;
        if (encoder2Pos != lastEncoder2Pos) {
        int increment2 = (encoder2Pos > lastEncoder2Pos) ? 1 : -1;
        modifyVariable(volume, increment2, 0.02, 0.00, 1.00);
        ampLeft.gain(volume);
        ampRight.gain(volume);
        Serial.println(encoder2Pos);
        lastEncoder2Pos = encoder2Pos;
        }
    }
  }

//*************************  Comportement des Boutons et LEDS  *********************************
  // Gestion Appui Switchs 3 à 5 en mode Latch
  if (pushButton3.fallingEdge()) {
    if (!button3State) {
      button3State = true;
    } else {
      button3State = false;
    }
  }
  if (pushButton4.fallingEdge()) {
    if (!button4State) {
      button4State = true;
    } else {
      button4State = false;
    }
  }
  if (pushButton5.fallingEdge()) {
    updateTapTempo();
  }
  // buttonState (Latch) > Led 1 to 2
  if (button3State == true) {
    digitalWrite(ledGreen1, LOW);
    Amixer.gain(6, 0, rvbToi2s);
    Amixer.gain(7, 1, rvbToi2s);
    Amixer.gain(6, 2, rvbToUsb);
    Amixer.gain(7, 3, rvbToUsb);
  } else {
    digitalWrite(ledGreen1, HIGH);
    Amixer.gain(6, 0, 0.0);
    Amixer.gain(7, 1, 0.0);
    Amixer.gain(6, 2, 0.0);
    Amixer.gain(7, 3, 0.0);
  }
  if (button4State == true) {
    digitalWrite(ledGreen2, LOW);
    Amixer.gain(8, 0, delay1Toi2s);
    Amixer.gain(8, 1, delay1Toi2s);
    Amixer.gain(9, 0, delay2Toi2s);
    Amixer.gain(9, 1, delay2Toi2s);
    Amixer.gain(8, 2, delay1ToUsb);
    Amixer.gain(8, 3, delay1ToUsb);
    Amixer.gain(9, 2, delay2ToUsb);
    Amixer.gain(9, 3, delay2ToUsb);
  } else {
    digitalWrite(ledGreen2, HIGH);
    Amixer.gain(8, 0, 0.0);
    Amixer.gain(8, 1, 0.0);
    Amixer.gain(9, 0, 0.0);
    Amixer.gain(9, 1, 0.0);
    Amixer.gain(8, 2, 0.0);
    Amixer.gain(8, 3, 0.0);
    Amixer.gain(9, 2, 0.0);
    Amixer.gain(9, 3, 0.0);
  }
}

//*********************************************************************************
//******************       FONCTIONS  D' AFFICHAGE    *****************************
//*********************************************************************************
void modeVolumeSelect() {                   // Selection Mode Volume
  mainMode = false;
  inMenu = false;
  drawVolumePage();
}
void modeFxSelect() {                       // Selection Mode Fx
  mainMode = true;
  inMenu = false;
  drawFxPage();
}
void drawVolumePage() {                     // Affichage page d'accueil VOLUME
  display.clear();
  display.drawXbm(0, 0, 128, 64, volumeBitmap);
  display.drawString(0, 0, "Matrix_v1");
  int progress = volume * 100;
  display.drawProgressBar(4, 45, 120, 15, progress);
  display.display();
}
                 
void drawFxPage() {                 // Affichage page d'accueil FXs
  display.clear();

  Page& page = pages[currentPage];
  display.setFont(ArialMT_Plain_10);

  // Titre au centre
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, page.name);

  // Affichage des paramètres
  for (int i = 0; i < 6; ++i) {
    int x = (i % 2 == 0) ? 0 : 68;
    int y = 10 + (i / 2) * 18; // int y = 12 + (i / 2) * 18
    Param* p = page.params[i];

    if (p == nullptr) {
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(x, y, "-");
      display.drawProgressBar(x, y + 10, 60, 6, 0); // (x, y + 10, 60, 8, 0)
    } else {
      bool isSelected = (i == selectedParam);
      if (isSelected) display.drawRect(x, y + 1, 64, 18); // (x - 2, y - 2, 64, 20)

      if (editing && isSelected) {
        //display.setColor(INVERSE); // BLACK
        //display.fillRect(x - 2, y - 2, 64, 20);
        //display.setColor(BLACK); // WHITE
      }

      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(x, y, p->label);

      float val = *(p->value);
      uint8_t percent = (p->type == FLOAT_0_1) ? (val * 100) : (val / 2000.0 * 100);
      display.drawProgressBar(x, y + 10, 60, 6, percent);

      if (editing && isSelected) display.setColor(WHITE); // reset
    }
  }

  display.display();
}

int countNonNullParams(Page& page) {
  int count = 0;
  for (int i = 0; i < 6; ++i)
    if (page.params[i] != nullptr) count++;
  return count;
}

void drawMenu() {                           // Affichage du menu
  display.clear();

  // Scroll dynamique
  if (selectedIndex < menuOffset) {
    menuOffset = selectedIndex;
  } else if (selectedIndex >= menuOffset + visibleLines) {
    menuOffset = selectedIndex - visibleLines + 1;
  }

  for (int i = 0; i < visibleLines; i++) {
    int itemIndex = i + menuOffset;
    if (itemIndex >= currentMenuLength) break;

    int y = i * lineHeight;
    String label = currentMenu[itemIndex].label;

    if (currentMenu[itemIndex].type == FLOAT_EDIT && currentMenu[itemIndex].floatVar != nullptr) {
      label += " : " + String(*currentMenu[itemIndex].floatVar, 2);
    } else if (currentMenu[itemIndex].type == FLOAT_LARGE_EDIT && currentMenu[itemIndex].floatVar != nullptr) {
      label += " : " + String(*currentMenu[itemIndex].floatVar, 2);
    }

    if (itemIndex == selectedIndex) {
      display.drawString(0, y, "> ");
      if (editingFloat || editingFloatLarge) {
        display.drawRect(0, y + 2, 128, lineHeight);
      }
    } else {
      display.drawString(0, y, "  ");
    }

    display.drawString(12, y, label);
  }

  display.display();
}

void onButtonPress() {                             // Entrée dans le menu
  if (!inMenu) {
    inMenu = true;
    selectedIndex = 0;
    menuOffset = 0;
    encoder.write(0);
  } else if (editingFloat || editingFloatLarge) {
    editingFloat = false;
    editingFloatLarge = false;
    encoder.write(selectedIndex * 4);
  } else {
    MenuItem selected = currentMenu[selectedIndex];
    switch (selected.type) {
      case ACTION:
        if (selected.label == "Quitter" || selected.label == "Retour") {
          if (stackIndex > 0) {
            stackIndex--;
            currentMenu = menuStack[stackIndex];
            currentMenuLength = menuLengths[stackIndex];
          } else {
            inMenu = false;
          } 
          selectedIndex = 0;
          menuOffset = 0;
          encoder.write(0);
        } else if (selected.actionFunc != nullptr) {
          selected.actionFunc();                  // <--- exécute la fonction associée // Sans argument
        } else if (selected.actionFuncWithArg && selected.argStr) {
          selected.actionFuncWithArg(selected.argStr);  // <--- exécute la fonction associée // Avec Argument
        }
        break;
      case SUBMENU:
        if (stackIndex < maxMenuDepth) {
          menuStack[stackIndex] = currentMenu;
          menuLengths[stackIndex] = currentMenuLength;
          stackIndex++;
          currentMenu = selected.subMenu;
          currentMenuLength = selected.numSubItems;
          selectedIndex = 0;
          menuOffset = 0;
          encoder.write(0);
        }
        break;
      case FLOAT_EDIT:
        editingFloat = true;
        encoder.write(*selected.floatVar * 1000); // encode en int ????
        Serial.println("*selected.floatVar * 1000");
        Serial.println(*selected.floatVar * 1000);
        break;
      case FLOAT_LARGE_EDIT:
        editingFloatLarge = true;
        encoder.write(*selected.floatVar * 1000); // encode en int ????
        break;
    }
  }
  int procUse = AudioProcessorUsage();
  int memUse = AudioMemoryUsage();
  Serial.printf("%i: proc %i, mem %i", procUse, memUse);
}
//*********************************************************************************
//***************       FONCTIONS  POUR VARIABLES  AUDIO    ***********************
//*********************************************************************************
void modifyVariable(float &variable, int increment, float step, float minVal, float maxVal) { // Modifier les variables affichées dans le menu
  variable = constrain(variable + increment * step, minVal, maxVal);
}
void executeVolume() {
  ampLeft.gain(volume);
  ampRight.gain(volume);
}
void executeMatrix() {
  Amixer.gain(0, 0, i2sToi2s);
  Amixer.gain(1, 1, i2sToi2s);
  Amixer.gain(0, 2, i2sToUsb);
  Amixer.gain(1, 3, i2sToUsb);

  Amixer.gain(2, 0, btToi2s);
  Amixer.gain(3, 1, btToi2s);
  Amixer.gain(2, 2, btToUsb);
  Amixer.gain(3, 3, btToUsb);

  Amixer.gain(4, 0, usbToi2s);
  Amixer.gain(5, 1, usbToi2s);
  Amixer.gain(4, 2, usbToUsb);
  Amixer.gain(5, 3, usbToUsb);
}
void executeReverb() {

  freeverbs1.roomsize(roomSize);
  freeverbs1.damping(damping);

  Amixer.gain(0, 4, i2sToRvb);
  Amixer.gain(1, 4, i2sToRvb);
  Amixer.gain(2, 4, btToRvb);
  Amixer.gain(3, 4, btToRvb);
  Amixer.gain(4, 4, usbToRvb);
  Amixer.gain(5, 4, usbToRvb);

  Amixer.gain(6, 0, rvbToi2s);
  Amixer.gain(7, 1, rvbToi2s);
  Amixer.gain(6, 2, rvbToUsb);
  Amixer.gain(7, 3, rvbToUsb);
  Amixer.gain(6, 4, rvbToRvb);
  Amixer.gain(7, 4, rvbToRvb);
  Amixer.gain(6, 5, rvbToDelay);
  Amixer.gain(7, 5, rvbToDelay);
}
void executeDelay() {
  delay1.delay(0, cleanDelay);
  delay1.delay(1, cleanDelay2);

  Amixer.gain(0, 5, i2sToDelay);
  Amixer.gain(1, 5, i2sToDelay);
  Amixer.gain(2, 5, btToDelay);
  Amixer.gain(3, 5, btToDelay);
  Amixer.gain(4, 5, usbToDelay);
  Amixer.gain(5, 5, usbToDelay);

  Amixer.gain(8, 0, delay1Toi2s);
  Amixer.gain(8, 1, delay1Toi2s);
  Amixer.gain(8, 2, delay1ToUsb);
  Amixer.gain(8, 3, delay1ToUsb);
  Amixer.gain(8, 4, delay1ToRvb);
  Amixer.gain(8, 5, delay1ToDelay);

  Amixer.gain(9, 0, delay2Toi2s);
  Amixer.gain(9, 1, delay2Toi2s);
  Amixer.gain(9, 2, delay2ToUsb);
  Amixer.gain(9, 3, delay2ToUsb);
  Amixer.gain(9, 4, delay2ToRvb);
  Amixer.gain(9, 5, delay2ToDelay);
}
void executeTone() {
  tone1.frequency(toneFrequency);

  Amixer.gain(10, 0, toneToi2s);
  Amixer.gain(10, 1, toneToi2s);
  Amixer.gain(10, 2, toneToUsb);
  Amixer.gain(10, 3, toneToUsb);
  Amixer.gain(10, 4, toneToRvb);
  Amixer.gain(10, 5, toneToDelay);
}
void executeAllVariables() {        // Appliquer la valeur de toutes les variables à leurs paramètres

  ampLeft.gain(volume);
  ampRight.gain(volume);
  
  Amixer.gain(0, 0, i2sToi2s);
  Amixer.gain(1, 1, i2sToi2s);
  Amixer.gain(0, 2, i2sToUsb);
  Amixer.gain(1, 3, i2sToUsb);
  Amixer.gain(2, 0, btToi2s);
  Amixer.gain(3, 1, btToi2s);
  Amixer.gain(2, 2, btToUsb);
  Amixer.gain(3, 3, btToUsb);
  Amixer.gain(4, 0, usbToi2s);
  Amixer.gain(5, 1, usbToi2s);
  Amixer.gain(4, 2, usbToUsb);
  Amixer.gain(5, 3, usbToUsb);

  freeverbs1.roomsize(roomSize);
  freeverbs1.damping(damping);
  Amixer.gain(0, 4, i2sToRvb);
  Amixer.gain(1, 4, i2sToRvb);
  Amixer.gain(2, 4, btToRvb);
  Amixer.gain(3, 4, btToRvb);
  Amixer.gain(4, 4, usbToRvb);
  Amixer.gain(5, 4, usbToRvb);
  Amixer.gain(6, 0, rvbToi2s);
  Amixer.gain(7, 1, rvbToi2s);
  Amixer.gain(6, 2, rvbToUsb);
  Amixer.gain(7, 3, rvbToUsb);
  Amixer.gain(6, 4, rvbToRvb);
  Amixer.gain(7, 4, rvbToRvb);
  Amixer.gain(6, 5, rvbToDelay);
  Amixer.gain(7, 5, rvbToDelay);

  delay1.delay(0, cleanDelay);
  delay1.delay(1, cleanDelay2);
  Amixer.gain(0, 5, i2sToDelay);
  Amixer.gain(1, 5, i2sToDelay);
  Amixer.gain(2, 5, btToDelay);
  Amixer.gain(3, 5, btToDelay);
  Amixer.gain(4, 5, usbToDelay);
  Amixer.gain(5, 5, usbToDelay);
  Amixer.gain(8, 0, delay1Toi2s);
  Amixer.gain(8, 1, delay1Toi2s);
  Amixer.gain(8, 2, delay1ToUsb);
  Amixer.gain(8, 3, delay1ToUsb);
  Amixer.gain(8, 4, delay1ToRvb);
  Amixer.gain(8, 5, delay1ToDelay);
  Amixer.gain(9, 0, delay2Toi2s);
  Amixer.gain(9, 1, delay2Toi2s);
  Amixer.gain(9, 2, delay2ToUsb);
  Amixer.gain(9, 3, delay2ToUsb);
  Amixer.gain(9, 4, delay2ToRvb);
  Amixer.gain(9, 5, delay2ToDelay);

  tone1.frequency(toneFrequency);
  Amixer.gain(10, 0, toneToi2s);
  Amixer.gain(10, 1, toneToi2s);
  Amixer.gain(10, 2, toneToUsb);
  Amixer.gain(10, 3, toneToUsb);
  Amixer.gain(10, 4, toneToRvb);
  Amixer.gain(10, 5, toneToDelay);
}
void executeAllStaticVariables() {
  delay1.disable(2);
  delay1.disable(3);
  delay1.disable(4);
  delay1.disable(5);
  delay1.disable(6);
  delay1.disable(7);

  tone1.amplitude(1.0);
  tone1.phase(0);
}
//*********************************************************************************
//***********************   INIT VARIABLES MATRIX     *****************************
//*********************************************************************************
void executeAmixerSetup() {
  Amixer.gain(0, 0, 0);
  Amixer.gain(0, 1, 0);
  Amixer.gain(0, 2, 0);
  Amixer.gain(0, 3, 0);
  Amixer.gain(0, 4, 0);
  Amixer.gain(0, 5, 0);
  Amixer.gain(0, 6, 0);
  Amixer.gain(0, 7, 0);
  Amixer.gain(0, 8, 0);
  Amixer.gain(0, 9, 0);
  Amixer.gain(0, 10, 0);
  Amixer.gain(0, 11, 0);
  Amixer.gain(0, 12, 0);
  Amixer.gain(0, 13, 0);
  Amixer.gain(0, 14, 0);
  Amixer.gain(0, 15, 0);

  Amixer.gain(1, 0, 0);
  Amixer.gain(1, 1, 0);
  Amixer.gain(1, 2, 0);
  Amixer.gain(1, 3, 0);
  Amixer.gain(1, 4, 0);
  Amixer.gain(1, 5, 0);
  Amixer.gain(1, 6, 0);
  Amixer.gain(1, 7, 0);
  Amixer.gain(1, 8, 0);
  Amixer.gain(1, 9, 0);
  Amixer.gain(1, 10, 0);
  Amixer.gain(1, 11, 0);
  Amixer.gain(1, 12, 0);
  Amixer.gain(1, 13, 0);
  Amixer.gain(1, 14, 0);
  Amixer.gain(1, 15, 0);

  Amixer.gain(2, 0, 0);
  Amixer.gain(2, 1, 0);
  Amixer.gain(2, 2, 0);
  Amixer.gain(2, 3, 0);
  Amixer.gain(2, 4, 0);
  Amixer.gain(2, 5, 0);
  Amixer.gain(2, 6, 0);
  Amixer.gain(2, 7, 0);
  Amixer.gain(2, 8, 0);
  Amixer.gain(2, 9, 0);
  Amixer.gain(2, 10, 0);
  Amixer.gain(2, 11, 0);
  Amixer.gain(2, 12, 0);
  Amixer.gain(2, 13, 0);
  Amixer.gain(2, 14, 0);
  Amixer.gain(2, 15, 0);

  Amixer.gain(3, 0, 0);
  Amixer.gain(3, 1, 0);
  Amixer.gain(3, 2, 0);
  Amixer.gain(3, 3, 0);
  Amixer.gain(3, 4, 0);
  Amixer.gain(3, 5, 0);
  Amixer.gain(3, 6, 0);
  Amixer.gain(3, 7, 0);
  Amixer.gain(3, 8, 0);
  Amixer.gain(3, 9, 0);
  Amixer.gain(3, 10, 0);
  Amixer.gain(3, 11, 0);
  Amixer.gain(3, 12, 0);
  Amixer.gain(3, 13, 0);
  Amixer.gain(3, 14, 0);
  Amixer.gain(3, 15, 0);

  Amixer.gain(4, 0, 0);
  Amixer.gain(4, 1, 0);
  Amixer.gain(4, 2, 0);
  Amixer.gain(4, 3, 0);
  Amixer.gain(4, 4, 0);
  Amixer.gain(4, 5, 0);
  Amixer.gain(4, 6, 0);
  Amixer.gain(4, 7, 0);
  Amixer.gain(4, 8, 0);
  Amixer.gain(4, 9, 0);
  Amixer.gain(4, 10, 0);
  Amixer.gain(4, 11, 0);
  Amixer.gain(4, 12, 0);
  Amixer.gain(4, 13, 0);
  Amixer.gain(4, 14, 0);
  Amixer.gain(4, 15, 0);

  Amixer.gain(5, 0, 0);
  Amixer.gain(5, 1, 0);
  Amixer.gain(5, 2, 0);
  Amixer.gain(5, 3, 0);
  Amixer.gain(5, 4, 0);
  Amixer.gain(5, 5, 0);
  Amixer.gain(5, 6, 0);
  Amixer.gain(5, 7, 0);
  Amixer.gain(5, 8, 0);
  Amixer.gain(5, 9, 0);
  Amixer.gain(5, 10, 0);
  Amixer.gain(5, 11, 0);
  Amixer.gain(5, 12, 0);
  Amixer.gain(5, 13, 0);
  Amixer.gain(5, 14, 0);
  Amixer.gain(5, 15, 0);

  Amixer.gain(6, 0, 0);
  Amixer.gain(6, 1, 0);
  Amixer.gain(6, 2, 0);
  Amixer.gain(6, 3, 0);
  Amixer.gain(6, 4, 0);
  Amixer.gain(6, 5, 0);
  Amixer.gain(6, 6, 0);
  Amixer.gain(6, 7, 0);
  Amixer.gain(6, 8, 0);
  Amixer.gain(6, 9, 0);
  Amixer.gain(6, 10, 0);
  Amixer.gain(6, 11, 0);
  Amixer.gain(6, 12, 0);
  Amixer.gain(6, 13, 0);
  Amixer.gain(6, 14, 0);
  Amixer.gain(6, 15, 0);

  Amixer.gain(7, 0, 0);
  Amixer.gain(7, 1, 0);
  Amixer.gain(7, 2, 0);
  Amixer.gain(7, 3, 0);
  Amixer.gain(7, 4, 0);
  Amixer.gain(7, 5, 0);
  Amixer.gain(7, 6, 0);
  Amixer.gain(7, 7, 0);
  Amixer.gain(7, 8, 0);
  Amixer.gain(7, 9, 0);
  Amixer.gain(7, 10, 0);
  Amixer.gain(7, 11, 0);
  Amixer.gain(7, 12, 0);
  Amixer.gain(7, 13, 0);
  Amixer.gain(7, 14, 0);
  Amixer.gain(7, 15, 0);

  Amixer.gain(8, 0, 0);
  Amixer.gain(8, 1, 0);
  Amixer.gain(8, 2, 0);
  Amixer.gain(8, 3, 0);
  Amixer.gain(8, 4, 0);
  Amixer.gain(8, 5, 0);
  Amixer.gain(8, 6, 0);
  Amixer.gain(8, 7, 0);
  Amixer.gain(8, 8, 0);
  Amixer.gain(8, 9, 0);
  Amixer.gain(8, 10, 0);
  Amixer.gain(8, 11, 0);
  Amixer.gain(8, 12, 0);
  Amixer.gain(8, 13, 0);
  Amixer.gain(8, 14, 0);
  Amixer.gain(8, 15, 0);

  Amixer.gain(9, 0, 0);
  Amixer.gain(9, 1, 0);
  Amixer.gain(9, 2, 0);
  Amixer.gain(9, 3, 0);
  Amixer.gain(9, 4, 0);
  Amixer.gain(9, 5, 0);
  Amixer.gain(9, 6, 0);
  Amixer.gain(9, 7, 0);
  Amixer.gain(9, 8, 0);
  Amixer.gain(9, 9, 0);
  Amixer.gain(9, 10, 0);
  Amixer.gain(9, 11, 0);
  Amixer.gain(9, 12, 0);
  Amixer.gain(9, 13, 0);
  Amixer.gain(9, 14, 0);
  Amixer.gain(9, 15, 0);
  
  Amixer.gain(10, 0, 0);
  Amixer.gain(10, 1, 0);
  Amixer.gain(10, 2, 0);
  Amixer.gain(10, 3, 0);
  Amixer.gain(10, 4, 0);
  Amixer.gain(10, 5, 0);
  Amixer.gain(10, 6, 0);
  Amixer.gain(10, 7, 0);
  Amixer.gain(10, 8, 0);
  Amixer.gain(10, 9, 0);
  Amixer.gain(10, 10, 0);
  Amixer.gain(10, 11, 0);
  Amixer.gain(10, 12, 0);
  Amixer.gain(10, 13, 0);
  Amixer.gain(10, 14, 0);
  Amixer.gain(10, 15, 0);

  Amixer.gain(11, 0, 0);
  Amixer.gain(11, 1, 0);
  Amixer.gain(11, 2, 0);
  Amixer.gain(11, 3, 0);
  Amixer.gain(11, 4, 0);
  Amixer.gain(11, 5, 0);
  Amixer.gain(11, 6, 0);
  Amixer.gain(11, 7, 0);
  Amixer.gain(11, 8, 0);
  Amixer.gain(11, 9, 0);
  Amixer.gain(11, 10, 0);
  Amixer.gain(11, 11, 0);
  Amixer.gain(11, 12, 0);
  Amixer.gain(11, 13, 0);
  Amixer.gain(11, 14, 0);
  Amixer.gain(11, 15, 0);

  Amixer.gain(12, 0, 0);
  Amixer.gain(12, 1, 0);
  Amixer.gain(12, 2, 0);
  Amixer.gain(12, 3, 0);
  Amixer.gain(12, 4, 0);
  Amixer.gain(12, 5, 0);
  Amixer.gain(12, 6, 0);
  Amixer.gain(12, 7, 0);
  Amixer.gain(12, 8, 0);
  Amixer.gain(12, 9, 0);
  Amixer.gain(12, 10, 0);
  Amixer.gain(12, 11, 0);
  Amixer.gain(12, 12, 0);
  Amixer.gain(12, 13, 0);
  Amixer.gain(12, 14, 0);
  Amixer.gain(12, 15, 0);

  Amixer.gain(13, 0, 0);
  Amixer.gain(13, 1, 0);
  Amixer.gain(13, 2, 0);
  Amixer.gain(13, 3, 0);
  Amixer.gain(13, 4, 0);
  Amixer.gain(13, 5, 0);
  Amixer.gain(13, 6, 0);
  Amixer.gain(13, 7, 0);
  Amixer.gain(13, 8, 0);
  Amixer.gain(13, 9, 0);
  Amixer.gain(13, 10, 0);
  Amixer.gain(13, 11, 0);
  Amixer.gain(13, 12, 0);
  Amixer.gain(13, 13, 0);
  Amixer.gain(13, 14, 0);
  Amixer.gain(13, 15, 0);

  Amixer.gain(14, 0, 0);
  Amixer.gain(14, 1, 0);
  Amixer.gain(14, 2, 0);
  Amixer.gain(14, 3, 0);
  Amixer.gain(14, 4, 0);
  Amixer.gain(14, 5, 0);
  Amixer.gain(14, 6, 0);
  Amixer.gain(14, 7, 0);
  Amixer.gain(14, 8, 0);
  Amixer.gain(14, 9, 0);
  Amixer.gain(14, 10, 0);
  Amixer.gain(14, 11, 0);
  Amixer.gain(14, 12, 0);
  Amixer.gain(14, 13, 0);
  Amixer.gain(14, 14, 0);
  Amixer.gain(14, 15, 0);

  Amixer.gain(15, 0, 0);
  Amixer.gain(15, 1, 0);
  Amixer.gain(15, 2, 0);
  Amixer.gain(15, 3, 0);
  Amixer.gain(15, 4, 0);
  Amixer.gain(15, 5, 0);
  Amixer.gain(15, 6, 0);
  Amixer.gain(15, 7, 0);
  Amixer.gain(15, 8, 0);
  Amixer.gain(15, 9, 0);
  Amixer.gain(15, 10, 0);
  Amixer.gain(15, 11, 0);
  Amixer.gain(15, 12, 0);
  Amixer.gain(15, 13, 0);
  Amixer.gain(15, 14, 0);
  Amixer.gain(15, 15, 0);
}
//*********************************************************************************
//******************       FONCTIONS    AUDIO          ****************************
//*********************************************************************************
void updateTapTempo() {
  unsigned long now = millis();
  prevTapTime = lastTapTime;
  lastTapTime = now;

  if (prevTapTime > 0) {
    unsigned long interval = lastTapTime - prevTapTime;
    if (interval >= 100 && interval <= 3000) {
      tapTempoMs = (float)interval;
      tempoValid = true;
      tapTempoBpm = 60000.0 / (float)tapTempoMs;

      applyDelayTime();
    } else {
      tempoValid = false;
      Serial.println("Tap ignoré (hors intervalle)");
    }
  }
}

void applyDelayTime() {
  float appliedMs = tapTempoMs * subdivisions[currentSubdivision];
  if (appliedMs > 3000) appliedMs = 3000;  // limitation du delay
  if (appliedMs < 10) appliedMs = 10;

  Serial.printf("Tempo: %.1f ms (%.1f BPM), Subdiv: %s → Delay: %.1f ms\n",
    tapTempoMs,
    60000.0 / tapTempoMs,
    subdivisionNames[currentSubdivision],
    appliedMs
  );
  cleanDelay = appliedMs;
  delay1.delay(0, cleanDelay);
  cleanDelay2 = appliedMs / 2;
  delay1.delay(1, cleanDelay2);
}

void cycleSubdivision() {
  currentSubdivision++;
  if (currentSubdivision >= (int)(sizeof(subdivisions) / sizeof(subdivisions[0]))) {
    currentSubdivision = 0;
  }
  if (tempoValid) {
    applyDelayTime(); // Réappliquer avec la nouvelle subdivision
  }
}
//*********************************************************************************
//***********************       FONCTIONS  ANNEXES    *****************************
//*********************************************************************************
void resetEsp() {
  digitalWrite(resetPin, LOW); // Reset l'esp32
  delay(250);
  digitalWrite(resetPin, HIGH);
}
void lightLed1() {
    if (button3State == false) {
    button3State = true;
    } else {
    button3State = false;
  }
}
void lightLed2() {
    if (button4State == false) {
    button4State = true;
    } else {
    button4State = false;
  }
}
void bpmLed3() {
  unsigned long now = millis();
  if (now - lastBeatTime >= tapTempoMs) {
    lastBeatTime = now;
    digitalWrite(ledGreen3, LOW);
    led3State = true;
  }

  // Éteindre la LED après 50 ms pour un petit flash
  if (led3State && (now - lastBeatTime >= 50)) {
    digitalWrite(ledGreen3, HIGH);
    led3State = false;
  }

}
//*********************************************************************************
//******************       FONCTIONS  DE  SNAPSHOTS    ****************************
//*********************************************************************************
void saveConfig(const char* fileNameToSave) {       // Sauvegarder la Config
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove(fileNameToSave);

  // Allocate a temporary JsonDocument
  //StaticJsonDocument<256> config;
  JsonDocument configToSave;

    // Set the values in the document
    configToSave["mainMode"] = mainMode;
    configToSave["button3State"] = button3State;
    configToSave["button4State"] = button4State;
    configToSave["volume"] = volume;
    configToSave["roomSize"] = roomSize;
    configToSave["damping"] = damping;
    configToSave["cleanDelay"] = cleanDelay;
    configToSave["cleanDelay2"] = cleanDelay2;
    configToSave["toneFrequency"] = toneFrequency;
    configToSave["tapTempoMs"] = tapTempoMs;
    configToSave["tapTempoBpm"] = tapTempoBpm;
    configToSave["currentSubdivision"] = currentSubdivision;

    configToSave["i2sToi2s"] = i2sToi2s;
    configToSave["i2sToUsb"] = i2sToUsb;
    configToSave["i2sToRvb"] = i2sToRvb;
    configToSave["i2sToDelay"] = i2sToDelay;
    configToSave["btToi2s"] = btToi2s;
    configToSave["btToUsb"] = btToUsb;
    configToSave["btToRvb"] = btToRvb;
    configToSave["btToDelay"] = btToDelay;
    configToSave["usbToi2s"] = usbToi2s;
    configToSave["usbToUsb"] = usbToUsb;
    configToSave["usbToRvb"] = usbToRvb;
    configToSave["usbToDelay"] = usbToDelay;
    configToSave["rvbToi2s"] = rvbToi2s;
    configToSave["rvbToUsb"] = rvbToUsb;
    configToSave["rvbToRvb"] = rvbToRvb;
    configToSave["rvbToDelay"] = rvbToDelay;
    configToSave["delay1Toi2s"] = delay1Toi2s;
    configToSave["delay1ToUsb"] = delay1ToUsb;
    configToSave["delay1ToRvb"] = delay1ToRvb;
    configToSave["delay1ToDelay"] = delay1ToDelay;
    configToSave["delay2Toi2s"] = delay2Toi2s;
    configToSave["delay2ToUsb"] = delay2ToUsb;
    configToSave["delay2ToRvb"] = delay2ToRvb;
    configToSave["delay2ToDelay"] = delay2ToDelay;
    configToSave["toneToi2s"] = toneToi2s;
    configToSave["toneToUsb"] = toneToUsb;
    configToSave["toneToRvb"] = toneToRvb;
    configToSave["toneToDelay"] = toneToDelay;

  display.clear();
  display.drawString(0, 20, "Sauvegarde...");
  display.display();

  // Ouvrir ou créer le fichier config.json
  File configFile = SD.open(fileNameToSave, FILE_WRITE);
  if (configFile) {
    Serial.println("Fichier de configuration ouvert pour sauvegarde :");
    Serial.println(fileNameToSave);
    if (serializeJson(configToSave, configFile) == 0) {
      Serial.println("ERREUR : Impossible de sérialiser le JSON !");
    } else {
      Serial.println("Configuration JSON sérialisée avec succès.");
    }
    configFile.close();
    Serial.println("Fichier config fermé.");
    display.clear();
    display.drawString(0, 20, "SAUVEGARDÉ !");
    display.display();
  } else {
    // Gestion d'erreur si le fichier ne peut pas être créé
    Serial.println("ERREUR : Impossible d'ouvrir ou de créer config !");
    display.clear();
    display.drawString(0, 20, "ERREUR SAUV !");
    display.display();
  }

  saveLastConfig(fileNameToSave);
  delay(1000);
}

void loadConfig(const char* fileNameToLoad) {     // Charger une config + appliquer les nouvelles valeurs des variables
  saveLastConfig(fileNameToLoad);
     // Affichage de la config chargée
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(20, 20, fileNameToLoad);
  display.setFont(ArialMT_Plain_10);
  display.display();
  delay(500);

  Serial.println("Début du chargement de la configuration...");
  File configFile = SD.open(fileNameToLoad);
  if (!configFile) {
    Serial.println("ERREUR : Impossible d'ouvrir config. Utilisation des valeurs par défaut.");
    return; // Quitte la fonction si le fichier n'existe pas
  }

  Serial.println("Fichier config ouvert pour lecture.");
  //StaticJsonDocument<256> config;
  JsonDocument configToLoad;

  DeserializationError error = deserializeJson(configToLoad, configFile);
  if (error) {
    Serial.print("ERREUR : Échec de la désérialisation du JSON : ");
    Serial.println(error.c_str());
    configFile.close();
    return; // Quitte la fonction si le JSON est invalide
  }

  Serial.println("JSON chargé avec succès. Attribution des variables...");
    mainMode = configToLoad["mainMode"] | false;
    button3State = configToLoad["button3State"] | false;
    button4State = configToLoad["button4State"] | false;
    volume = configToLoad["volume"] | 0.20; // Valeur par défaut : 0.20
    roomSize = configToLoad["roomSize"] | 0.50;
    damping = configToLoad["damping"] | 0.50;
    cleanDelay = configToLoad["cleanDelay"] | 1000;
    cleanDelay2 = configToLoad["cleanDelay2"] | 500;
    toneFrequency = configToLoad["toneFrequency"] | 1000;
    tapTempoMs = configToLoad["tapTempoMs"] | 1000;
    tapTempoBpm = configToLoad["tapTempoBpm"] | 120;
    currentSubdivision = configToLoad["currentSubdivision"] = 0;

    i2sToi2s = configToLoad["i2sToi2s"] | 0.00;
    i2sToUsb= configToLoad["i2sToUsb"] | 0.00;
    i2sToRvb= configToLoad["i2sToRvb"] | 0.00;
    i2sToDelay= configToLoad["i2sToDelay"] | 0.00;
    btToi2s= configToLoad["btToi2s"] | 0.00;
    btToUsb= configToLoad["btToUsb"] | 0.00;
    btToRvb= configToLoad["btToRvb"] | 0.00;
    btToDelay = configToLoad["btToDelay"] | 0.00;
    usbToi2s = configToLoad["usbToi2s"] | 0.00;
    usbToUsb = configToLoad["usbToUsb"] | 0.00;
    usbToRvb = configToLoad["usbToRvb"] | 0.00;
    usbToDelay = configToLoad["usbToDelay"] | 0.00;
    rvbToi2s = configToLoad["rvbToi2s"] | 0.00;
    rvbToUsb = configToLoad["rvbToUsb"] | 0.00;
    rvbToRvb = configToLoad["rvbToRvb"] | 0.00;
    rvbToDelay = configToLoad["rvbToDelay"] | 0.00;
    delay1Toi2s = configToLoad["delay1Toi2s"] | 0.00;
    delay1ToUsb = configToLoad["delay1ToUsb"] | 0.00;
    delay1ToRvb = configToLoad["delay1ToRvb"] | 0.00;
    delay1ToDelay = configToLoad["delay1ToDelay"] | 0.00;
    delay2Toi2s = configToLoad["delay2Toi2s"] | 0.00;
    delay2ToUsb = configToLoad["delay2ToUsb"] | 0.00;
    delay2ToRvb = configToLoad["delay2ToRvb"] | 0.00;
    delay2ToDelay = configToLoad["delay2ToDelay"] | 0.00;
    toneToi2s = configToLoad["toneToi2s"] | 0.00;
    toneToUsb = configToLoad["toneToUsb"] | 0.00;
    toneToRvb = configToLoad["toneToRvb"] | 0.00;
    toneToDelay = configToLoad["toneToDelay"] | 0.00;         

  configFile.close();
  Serial.println("Fichier config fermé ");

  executeAllVariables();

  display.clear();
  display.drawString(0, 20, "CONFIG CHARGÉE !");
  display.display();
  delay(500);
}

void saveLastConfig(const char* fileNameToSaveLast) {     // Sauvegarder le nom de la dernière config chargée / sauvegardée
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove("/lastConfig.json");

  // Open file for writing
  File file = SD.open("/lastConfig.json", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  JsonDocument saveLastConfig;

  // Set the values in the document
  saveLastConfig["lastConfig"] = fileNameToSaveLast;

  // Serialize JSON to file
  if (serializeJson(saveLastConfig, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();

  Serial.println("Mise à jour de la config utilisée :");
  Serial.println(fileNameToSaveLast);
}
const char* loadLastConfig() {                // Récupérer le nom de la dernière config utilisée
  // Open file for reading
  File file = SD.open("/lastConfig.json");

  // Allocate a temporary JsonDocument
  JsonDocument checkConfig;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(checkConfig, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  lastConfigToUse = checkConfig["lastConfig"];


  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  Serial.println("Dernière config utilisée :");
  Serial.println(lastConfigToUse);
  return lastConfigToUse;
}
void autoSave() {                         // Sauvegarde automatique de la config en cours d'utilisation
  unsigned long currentMillis2 = millis();

  if (currentMillis2 - previousMillis2 >= interval2)
  {
    // save the last time you save
    previousMillis2 = currentMillis2;

    saveConfig("/temp.json");

    Serial.println("Sauvegarde automatique dans /temp.json");
  }
}