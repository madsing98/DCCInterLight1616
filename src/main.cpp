/*************************************************************************************************************\
Mobile DCC decoder based on ATtiny1616 for Kato cars interior lights

Core / libraries
- megaTinyCore: https://github.com/SpenceKonde/megaTinyCore
- NmraDcc: https://github.com/mrrwa/NmraDcc

Hardware resources
- megaTinyCore uses TCA0 for PWM with AnalogWrite()
    - See page 182 of data sheet, 20.3.3.4.3 Single-Slope PWM Generation
    - megaTinyCore uses split-mode, to have up to six 8-bit PWM outputs (WO (Waveform Out) [0..5]).
    See 20.3.3.6 Split Mode - Two 8-Bit Timer/Counters
    - From the table below, without using multiplex signals, PWM could be (for ATtiny1616-MNR VQFN 20-pin)
        * WO0 - PB0 - pin 14
        * WO1 - PB1 - pin 13
        * WO2 - PB2 - pin 12
        * WO3 - PA3 - pin 2
        * WO4 - PA4 - pin 5
        * WO5 - PA5 - pin 6
    - From megaTinyCore source code
        #define digitalPinHasPWM(p)
            ((p) == PIN_PA4 || (p) == PIN_PA5 || (p) == PIN_PB2 || (p) == PIN_PB1 || (p) == PIN_PB0 || (p) == PIN_PA3)
- NmraDcc
    - Uses the INT0/1 Hardware Interrupt and micros() ONLY
    - On the ATtiny1616, millis() and micros() use TCD0

- EEPROM
    - The ATtiny1616 EEPROM size is 256 bytes, with addresses ranging from 0 to 255
    - NmraDcc uses the EEPROM to store CVs. CVs are stored at the location corresponding to the CV number
      (i.e., CV29 is stored at EEPROM location 29). So, with the ATtiny1616 and the NmraDcc library, CV numbers must be
      between 0 and 255
    - We will also use location 255 to store the status of the functions (F0 to F4). The goal is to have the lights
      in the correct state at power on, before the decoder receives any DCC packet setting these functions

CV Map
CV1     Primary Address
CV7     Manufacturer Version Number
CV8     Manufacturer ID Number
CV29    Mode Control

CV96    Light Brightness (0..255)
CV97    Light Temperature (0..255)
            0 = warm white
          128 = natural white (default)
          255 = cool white
CV98    Light Function control
          0 = F0
          1 = F1 (default)
          2 = F2
          3 = F3
          4 = F4
CV99    Computed Warm White LED Luminance (0..255). Can also be written directly for debugging purposes
CV100   Computed Cool White LED Luminance (0..255). Can also be written directly for debugging purposes
\*************************************************************************************************************/

#include <Arduino.h>
#include <NmraDcc.h>
#include <EEPROM.h>

// Uncomment to send debugging messages to the serial line
// #define DEBUG

// Versioning
const uint8_t versionIdMajor = 2;
const uint8_t versionIdMinor = 1;
const uint8_t versionId = versionIdMajor << 4 | versionIdMinor;

// Hardware pin definitions
const uint8_t numberOfLights = 2;
const pin_size_t pinLight[numberOfLights] = {PIN_PB0, PIN_PB1};
//                      Light # on the PCB      0        1
const uint8_t warmWhiteLight = 0;
const uint8_t coolWhiteLight = 1;
const pin_size_t pinDCCInput = PIN_PA2;

// Objects from NmraDcc
NmraDcc Dcc;

// fctsCache holds the current state (ON/OFF) of loco functions F0 to F4
// It is stored in the EEPROM at address fctsEepromAddress
uint8_t fctsCache = 0;
const uint16_t fctsEepromAddress = 255;
const uint8_t numberOfFunctions = 5;
const uint8_t fctBitMask[numberOfFunctions] = {FN_BIT_00, FN_BIT_01, FN_BIT_02, FN_BIT_03, FN_BIT_04};

// CV number definitions
const uint8_t CV0Check = 0;
const uint8_t CV1PrimaryAddress = 1;
const uint8_t CV7ManufacturerVersionNumber = 7;
const uint8_t CV8ManufacturerIDNumber = 8;
const uint8_t CV29ModeControl = 29;

// CVs related to light outputs
const uint8_t CV96LightBrightness = 96;
const uint8_t CV97LightTemperature = 97;
const uint8_t CV98LightFctCtrl = 98; // CV with the highest number

// Structure for CV Values Table and default CV Values table as required by NmraDcc for storing default values
uint8_t FactoryDefaultCVIndex = 0;

struct CVPair
{
    uint16_t CV;
    uint8_t Value;
};

const CVPair FactoryDefaultCVs[] =
    {
        {CV1PrimaryAddress, 3},
        {CV7ManufacturerVersionNumber, 1},
        {CV8ManufacturerIDNumber, 13},
        {CV29ModeControl, 0},

        {CV96LightBrightness, 20},
        {CV97LightTemperature, 128},
        {CV98LightFctCtrl, 1}};

void updateLights();

// This callback function is called when a CV Value changes so we can update the lights
void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG
    Serial.print("notifyCVChange: CV: ");
    Serial.print(CV);
    Serial.print(" Value: ");
    Serial.println(Value);
#endif

    updateLights();
}

// This callback function is called when the CVs must be reset to their factory defaults
// Make FactoryDefaultCVIndex non-zero and equal to the number of CVs to be reset
// to flag to the loop() function that a reset to factory defaults needs to be done
void notifyCVResetFactoryDefault()
{
#ifdef DEBUG
    Serial.println("notifyCVResetFactoryDefault");
#endif
    FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// This callback function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
    // Check that the DCC packet is for functions 0 to 4 (the only functions we use)
    // Check that one of the functions has changed
    if (FuncGrp == FN_0_4 && FuncState != fctsCache)
    {
#ifdef DEBUG
        Serial.print("Function Group: ");
        Serial.print(FuncGrp);
        Serial.print("|State = 0b");
        Serial.println(FuncState, BIN);
#endif
        fctsCache = FuncState;
        updateLights();
        EEPROM.write(fctsEepromAddress, fctsCache);
    }
}

// Restore the status of all functions from the EEPROM to the cache
void readFctsToCache()
{
    fctsCache = EEPROM.read(fctsEepromAddress);
}

// Returns true is the function "fctNumber" is on
bool checkFct(uint8_t fctNumber)
{
    return (fctsCache & fctBitMask[fctNumber]);
}

// Luminance tables for warm white and cool white LEDs
// These tables implement the gamma function required to convert the desired brightness of the LED into
// a luminance value used to drive the LED's PWM duty cycle
// Note:
//  - "Brightness" is the light intensity as perceived by the human eye
//  - "Luminance" is the measurable amount of light really emitted by the LED
const uint8_t warmWhiteLuminanceTable[] = {
    0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2, 
    2,   2,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   6,   6,   6, 
    7,   7,   7,   8,   8,   8,   9,   9,  10,  10,  10,  11,  11,  12,  12,  13, 
   13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21, 
   22,  22,  23,  23,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,  31, 
   32,  33,  34,  34,  35,  36,  37,  37,  38,  39,  40,  40,  41,  42,  43,  44, 
   44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58, 
   58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73, 
   74,  75,  76,  77,  78,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  91, 
   92,  93,  94,  95,  96,  97,  99, 100, 101, 102, 103, 105, 106, 107, 108, 109, 
  111, 112, 113, 114, 116, 117, 118, 120, 121, 122, 123, 125, 126, 127, 129, 130, 
  131, 133, 134, 135, 137, 138, 139, 141, 142, 144, 145, 146, 148, 149, 151, 152, 
  153, 155, 156, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 173, 174, 176, 
  177, 179, 180, 182, 183, 185, 186, 188, 190, 191, 193, 194, 196, 198, 199, 201, 
  202, 204, 206, 207, 209, 211, 212, 214, 216, 217, 219, 221, 222, 224, 226, 227, 
  229, 231, 233, 234, 236, 238, 240, 241, 243, 245, 247, 248, 250, 252, 254, 255
};

const uint8_t coolWhiteLuminanceTable[] = {
    0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2, 
    2,   2,   2,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   6, 
    6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11, 
   12,  12,  13,  13,  14,  14,  15,  15,  16,  16,  16,  17,  17,  18,  19,  19, 
   20,  20,  21,  21,  22,  22,  23,  24,  24,  25,  25,  26,  27,  27,  28,  28, 
   29,  30,  30,  31,  32,  32,  33,  34,  34,  35,  36,  36,  37,  38,  39,  39, 
   40,  41,  42,  42,  43,  44,  45,  45,  46,  47,  48,  49,  49,  50,  51,  52, 
   53,  54,  54,  55,  56,  57,  58,  59,  60,  61,  62,  62,  63,  64,  65,  66, 
   67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82, 
   83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  97,  98,  99, 
  100, 101, 102, 103, 104, 106, 107, 108, 109, 110, 111, 113, 114, 115, 116, 117, 
  118, 120, 121, 122, 123, 125, 126, 127, 128, 130, 131, 132, 133, 135, 136, 137, 
  138, 140, 141, 142, 144, 145, 146, 148, 149, 150, 152, 153, 154, 156, 157, 159, 
  160, 161, 163, 164, 165, 167, 168, 170, 171, 173, 174, 175, 177, 178, 180, 181, 
  183, 184, 186, 187, 189, 190, 192, 193, 195, 196, 198, 199, 201, 202, 204, 205, 
  207, 208, 210, 211, 213, 215, 216, 218, 219, 221, 222, 224, 226, 227, 229, 230
};

void updateLights()
{
    uint8_t warmWhiteLEDBrightness, coolWhiteLEDBrightness;

    // Process the value of light outputs
    // We use analogWrite() as all output pins support PWM
    if (checkFct(Dcc.getCV(CV98LightFctCtrl)))
    {
        // Note: C always performs arithmetic operations in the size of the largest involved datatype.
        // Here we cast the operands to uint16_t
        warmWhiteLEDBrightness = ((uint16_t)Dcc.getCV(CV96LightBrightness) * (255 - (uint16_t)Dcc.getCV(CV97LightTemperature))) / 256;
        coolWhiteLEDBrightness = ((uint16_t)Dcc.getCV(CV96LightBrightness) * (uint16_t)Dcc.getCV(CV97LightTemperature)) / 256;
        analogWrite(pinLight[warmWhiteLight], warmWhiteLuminanceTable[warmWhiteLEDBrightness]);
        analogWrite(pinLight[coolWhiteLight], coolWhiteLuminanceTable[coolWhiteLEDBrightness]);
#ifdef DEBUG
        Serial.print("Writing warmWhiteLEDBrightness: luminance[");
        Serial.print(warmWhiteLEDBrightness);
        Serial.print("] = ");
        Serial.println(warmWhiteLuminanceTable[warmWhiteLEDBrightness]);
        Serial.print("Writing coolWhiteLEDBrightness: luminance[");
        Serial.print(coolWhiteLEDBrightness);
        Serial.print("] = ");
        Serial.println(coolWhiteLuminanceTable[coolWhiteLEDBrightness]);
#endif
    }
    else
    {
        analogWrite(pinLight[warmWhiteLight], 0);
        analogWrite(pinLight[coolWhiteLight], 0);
    }
}

// This callback function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60mA current drain on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void)
{
#ifdef DEBUG
    Serial.println("notifyCVAck");
#endif

    digitalWrite(pinLight[coolWhiteLight], HIGH);
    delay(6);
    digitalWrite(pinLight[coolWhiteLight], LOW);
}

void setup()
{
    // Set light pins and DCC ACK pin to outputs
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
    {
        analogWrite(pinLight[lightNr], 0);
        pinMode(pinLight[lightNr], OUTPUT);
    }

#ifdef DEBUG
    // Serial TX used for debugging messages
    // Two mapping options for Serial are PB2, PB3, PB1, PB0 (default) and PA1, PA2, PA3, PA4 for TX, RX, XCK, XDIR.
    Serial.swap(); // Use the second set of serial pins. TX is on now PA1
    Serial.begin(115200);
    Serial.println();
    Serial.println("-- Starting tiny DCC interior light decoder --");
#endif

    readFctsToCache();

    // Initialize the NmraDcc library
    // void NmraDcc::pin (uint8_t ExtIntPinNum, uint8_t EnablePullup)
    // void NmraDcc::init (uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV)
    Dcc.pin(pinDCCInput, false);
    Dcc.init(MAN_ID_DIY, versionId, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

    // Commented out as not necessary. Uncomment for debugging purposes only. notifyCVResetFactoryDefault() is
    // automatically called at the very first call (i.e. unprogrammed EEPROM) of
    // NmraDcc::init() when FLAGS_AUTO_FACTORY_DEFAULT is set
    // notifyCVResetFactoryDefault();

    updateLights();
}

#ifdef DEBUG
uint32_t loopCounterLow = 200000;
uint32_t loopCounterHigh = 0;
#endif

void loop()
{
#ifdef DEBUG
    if (loopCounterLow == 200000)
    {
        loopCounterLow = 0;
        Serial.print("loop ");
        Serial.println(loopCounterHigh);
        loopCounterHigh++;
    }
    loopCounterLow++;
#endif

    // Process DCC packets
    Dcc.process();

    // Handle resetting CVs to Factory Defaults
    if (FactoryDefaultCVIndex && Dcc.isSetCVReady())
    {
        FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
        Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
    }
}