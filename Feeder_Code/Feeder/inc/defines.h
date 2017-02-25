/*
*  Hier werden grundlegenden Definitionen für einen Feeder gesetzt.
*/


// Schrittmotortreiber
#define DRIVE_PORT GPIOA
#define DRIVE_STEP GPIO1
#define DRIVE_DIR GPIO7
#define DRIVE_EN GPIO3
#define DRIVE_SLP GPIO4
#define DRIVE_RST GPIO5
#define DRIVE_MS_1 GPIO6
#define DRIVE_MS_2 GPIO8
#define DRIVE_MS_3 GPIO12

#define MICROSTEPPING_NONE        0
#define MICROSTEPPING_HALF        1
#define MICROSTEPPING_QUARTER     2
#define MICROSTEPPING_EIGHT       3
#define MICROSTEPPING_SIXTEENTH   7
#define STEPS_PER_DEGREE          100
#define DEFAULT_VMAX // °/s
#define DEFAULT_A // °/s²
#define DEFAULT_J // °/s³

// Reflexkoppler
#define REFLEX_LED_PORT GPIOA
#define REFLEX_LED_PIN GPIO10
#define REFLEX_INPUT_PORT GPIOA
#define REFLEX_INPUT_PIN GPIO11

// ADCs
#define ADC_POWERVCC
#define ADC_MOTORSTROM

// Thermofühler (1-Wire-Bus)
#define DS18B20_PORT
#define DS18B20_PIN


// Gammekorrektur für die LED-Farben
int gamma8[] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
        2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
        5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
       10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
       17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
       25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
       37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
       51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
       69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
       90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
      115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
      144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
      177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
      215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };


// Definitionen und Variablen für den LED-Teil:

#define LED_COUNT 5
#define TICK_NS (1000/48)
#define WS0 (350 / TICK_NS)
#define WS1 (800 / TICK_NS)
#define WSP (1300 / TICK_NS)
#define WSL (20000 / TICK_NS)
