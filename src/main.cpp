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

- CV19 Consist Address: When defined (not zero), speed and function commands respond at consist address, not
    at the primary DCC address.
    Example: For a train with motor decoder's address set to 32, set primary address of light decoder to 1032 and
    consist address to 32. All functions will respond at address 32, but use the address 1032 to set the light
    decoder's CVs

CV Map
CV1     Primary Address
CV7     Manufacturer Version Number
CV8     Manufacturer ID Number
CV17+18 Extended Address
CV19    Consist Address
CV29    Mode Control

CV1000  Light Brightness (0..255) (default: 50)
CV1001  Light CCT (Correlated Color Temperature) (0..255)
            0: warm white 3000K
          128: natural white
          255: cool white 6500K (default)
CV1002  Light Function control
          0: F0
          1: F1 (default)
          2: F2
          ...
          28: F28
CV1003  Light Brightness Set 2 (0..255) (default: 30)
CV1004  Light CCT (Correlated Color Temperature) Set 2 (0..255)
            0: warm white 3000K
          128: natural white
          255: cool white 6500K (default)
CV1005  Light Function control Set 2
          0: F0
          1: F1
          2: F2
          ...
          20: F20 (default)
          ...
          28: F28
          255: Not used
CV1010  Light Test
          0: CV40/CV41 contain light brightness and CCT (default)
          1: CV40/CV41 contain Warm White Luminance and Cool White Luminance (used for testing)
\*************************************************************************************************************/

#include <Arduino.h>
#include <NmraDcc.h>
#include <EEPROM.h>
#include "version.h"

// Uncomment to send debugging messages to the serial line
#define DEBUG

// Hardware pin definitions
const uint8_t numberOfLights = 2;
const pin_size_t pinLight[numberOfLights] = {PIN_PB0, PIN_PB1};
//                      Light # on the PCB      0        1
const uint8_t warmWhiteLight = 0;
const uint8_t coolWhiteLight = 1;
const pin_size_t pinDCCInput = PIN_PA2;

// Objects from NmraDcc
NmraDcc dcc;

// funcCache[] holds the current state (ON/OFF) of the 29 loco functions F0 to F28
// They are split into 5 groups (1 to 5, group 0 is not used), which are stored in the EEPROM from the address fctsEepromAddress
const uint8_t numberOfFunctionGroups = FN_LAST;
const uint8_t numberOfFunctions = 29;
uint8_t funcCache[numberOfFunctionGroups] = {0, 0, 0, 0, 0, 0};
const uint16_t fctsEepromAddress = 255 - numberOfFunctionGroups + 1;
const uint8_t funcBitMask[numberOfFunctions] = {FN_BIT_00, FN_BIT_01, FN_BIT_02, FN_BIT_03, FN_BIT_04,
                                                FN_BIT_05, FN_BIT_06, FN_BIT_07, FN_BIT_08,
                                                FN_BIT_09, FN_BIT_10, FN_BIT_11, FN_BIT_12,
                                                FN_BIT_13, FN_BIT_14, FN_BIT_15, FN_BIT_16, FN_BIT_17, FN_BIT_18, FN_BIT_19, FN_BIT_20,
                                                FN_BIT_21, FN_BIT_22, FN_BIT_23, FN_BIT_24, FN_BIT_25, FN_BIT_26, FN_BIT_27, FN_BIT_28};
const uint8_t funcGroup[numberOfFunctions] =   {FN_0_4, FN_0_4, FN_0_4, FN_0_4, FN_0_4,
                                                FN_5_8, FN_5_8, FN_5_8, FN_5_8, 
                                                FN_9_12, FN_9_12, FN_9_12, FN_9_12,
                                                FN_13_20, FN_13_20, FN_13_20, FN_13_20, FN_13_20, FN_13_20, FN_13_20, FN_13_20, 
                                                FN_21_28, FN_21_28, FN_21_28, FN_21_28, FN_21_28, FN_21_28, FN_21_28, FN_21_28};

// Struct and table for storing CV's data
struct cvStruct
{
    uint8_t cvIndex;        // CV index, only used as debug check of cvData[] table
    uint16_t cvNr;          // CV number as used by DCC
    bool applyDefault;      // True if the default value must be applied after a Factory Reset
    bool writable;          // True if the CV can be written. False if the CV is read only
    uint8_t defaultValue;   // Default value applied at first power on or after a Factory Reset
    uint8_t value;          // Current value cached in RAM
};

// Enum of all CVs, assigning them with their index number. This index is also the address in
// EEPROM where the CV will be stored
enum cvIndex
{
    cvPrimaryAddress,
    cvManufacturerVersionNumber,
    cvManufacturerIDNumber,
    cvExtendedAddressMSB,
    cvExtendedAddressLSB,
    cvConsistAddress,
    cvModeControl,
    cvLightBrightness,
    cvLightColorTemperature,
    cvLightFctCtrl,
    cvLightBrightness2,
    cvLightColorTemperature2,
    cvLightFctCtrl2,
    cvLightTest
};

struct cvStruct cvData[] =
{
//   cvIndex,        cvNr,applyDefault,defaultValue,value 
    {cvPrimaryAddress, 1, true, true, 3, 0},
    {cvManufacturerVersionNumber, 7, false, false, 0, 0},
    {cvManufacturerIDNumber, 8, false, false, 0, 0},
    {cvExtendedAddressMSB, 17, true, true, 0, 0},
    {cvExtendedAddressLSB, 18, true, true, 0, 0},
    {cvConsistAddress, 19, true, true, 0, 0},
    {cvModeControl, 29, true, true, 2, 0}, 
    {cvLightBrightness, 1000, true, true, 50, 0},
    {cvLightColorTemperature, 1001, true, true, 255, 0},
    {cvLightFctCtrl, 1002, true, true, 1, 0},
    {cvLightBrightness2, 1003, true, true, 30, 0},
    {cvLightColorTemperature2, 1004, true, true, 255, 0},
    {cvLightFctCtrl2, 1005, true, true, 20, 0},
    {cvLightTest, 1010, true, true, 0, 0}
};

const uint8_t nrCVs = sizeof(cvData) / sizeof(cvStruct);
uint8_t factoryDefaultCVIndex = 0;

void updateLights();

// This callback function is called when the decoder enters or exits service mode
// We update the lights at the end of the service mode
void notifyServiceMode(bool inServiceMode)
{
#ifdef DEBUG
    Serial.print("notifyServiceMode: inServiceMode: ");
    Serial.println(inServiceMode);
#endif

    if (inServiceMode)
    {
        digitalWrite(pinLight[warmWhiteLight], LOW);
        digitalWrite(pinLight[coolWhiteLight], LOW);
    }
    else
    {
        updateLights();
    }
}

// This callback function is called when the CVs must be reset to their factory defaults
// Make FactoryDefaultCVIndex non-zero and equal to the number of CVs to be reset
// to flag to the loop() function that a reset to factory defaults needs to be done
void notifyCVResetFactoryDefault()
{
#ifdef DEBUG
    Serial.println("notifyCVResetFactoryDefault");
#endif
    factoryDefaultCVIndex = nrCVs;
};

// This callback function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
    // Check that one of the functions has changed by comparing it to the cache
    if(FuncState != funcCache[FuncGrp])
    {
#ifdef DEBUG
        Serial.print("DCC Addr: ");
        Serial.print(Addr);
        Serial.print("|Function Group: ");
        Serial.print(FuncGrp);
        Serial.print("|State = 0b");
        Serial.println(FuncState, BIN);
#endif
        funcCache[FuncGrp] = FuncState;
        updateLights();
        EEPROM.put(fctsEepromAddress, funcCache);
    }
}

// This function is called when the library needs to determine if a CV is valid
uint8_t notifyCVValid(uint16_t CV, uint8_t Writable)
{
#ifdef DEBUG
    Serial.print("notifyCVValid: CV: ");
    Serial.print(CV);
    Serial.print(" Writable: ");
    Serial.println(Writable);
#endif

    for (uint8_t i = 0; i < nrCVs; i++)                 // Locate the CV in cvData[]
    {
        if (cvData[i].cvNr == CV)                       // Found it!
        {
            if (!Writable)                              // If we just have to check if the CV is readable
                return (1);                             // Return "yes"
            else                                        // If we also have to check if the CV is writable
                return (uint8_t)cvData[i].writable;     // Return the ".writable" data from cvData[]
        }
    }
    return 0;                                           // If we cannot find the CV, just return "no"
}

// This function is called when the library needs to read a CV
uint8_t notifyCVRead(uint16_t CV)
{
#ifdef DEBUG
    Serial.print("notifyCVRead: CV: ");
    Serial.print(CV);
#endif

    for (uint8_t i = 0; i < nrCVs; i++)                 // Locate the CV in cvData[]
    {
        if (cvData[i].cvNr == CV)                       // Found it!
        {
#ifdef DEBUG
            Serial.print(" Value: ");
            Serial.println(cvData[i].value);
#endif
            return cvData[i].value;                     // Return the cached value from cvData[]
        }
    }
}

// This function is called when the library needs to write a CV
uint8_t notifyCVWrite(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG
    Serial.print("notifyCVWrite: CV: ");
    Serial.print(CV);
    Serial.print(" Value: ");
    Serial.println(Value);
#endif

    for (uint8_t i = 0; i < nrCVs; i++)                 // Locate the CV in cvData[]
    {
        if (cvData[i].cvNr == CV)                       // Found it!
        {
            if (cvData[i].value != Value)               // If the new value is different than the cached value
            {
                EEPROM.write(i, Value);                 // Store the new value in EEPROM
                cvData[i].value = Value;                //   and in the cache
#ifdef DEBUG
                Serial.print("EEPROM.write: i: ");
                Serial.print(i);
                Serial.print(" Value: ");
                Serial.println(Value);
#endif
                updateLights();                         // We update all lights if any CV changes
            }
            return Value;                               // Return the value written
        }
    }
}

// Restore all CVs from the EEPROM to the cvData[] cache

void readCVsToCache()
{
    for (uint8_t i = 0; i < nrCVs; i++)
    {
        cvData[i].value = EEPROM.read(i);
    }

}

// Restore the status of all functions from the EEPROM to the cache
void readFuncsToCache()
{
    EEPROM.get(fctsEepromAddress, funcCache);
}

// Returns true is the function "funcNumber" is on
bool checkFunc(uint8_t funcNumber)
{
    if (funcNumber < numberOfFunctions)
        return (funcCache[funcGroup[funcNumber]] & funcBitMask[funcNumber]);
    else
        return false;
}

// Luminance tables for warm white and cool white LEDs
// These tables implement the gamma function required to convert the desired brightness of the LED into
// a luminance value used to drive the LED's PWM duty cycle
// Note:
//  - "Brightness" is the light intensity as perceived by the human eye
//  - "Luminance" is the measurable amount of light really emitted by the LED
// warmWhiteLuminanceTable[]: Gamma = 2.2, Output Range = 255
const uint8_t warmWhiteLuminanceTable[] = {
    0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
    1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3, 
    3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7, 
    7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  11,  12,  12, 
   13,  13,  14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20, 
   20,  21,  22,  22,  23,  23,  24,  24,  25,  26,  26,  27,  28,  28,  29,  30, 
   30,  31,  32,  32,  33,  34,  34,  35,  36,  37,  37,  38,  39,  40,  41,  41, 
   42,  43,  44,  45,  46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56, 
   56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  67,  68,  69,  70,  71,  72, 
   73,  74,  75,  76,  78,  79,  80,  81,  82,  83,  85,  86,  87,  88,  89,  91, 
   92,  93,  94,  96,  97,  98, 100, 101, 102, 104, 105, 106, 108, 109, 110, 112, 
  113, 115, 116, 118, 119, 120, 122, 123, 125, 126, 128, 129, 131, 132, 134, 136, 
  137, 139, 140, 142, 143, 145, 147, 148, 150, 152, 153, 155, 157, 158, 160, 162, 
  163, 165, 167, 169, 170, 172, 174, 176, 177, 179, 181, 183, 185, 187, 188, 190, 
  192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222, 
  224, 226, 228, 230, 232, 234, 236, 238, 240, 242, 245, 247, 249, 251, 253, 255
};

// coolWhiteLuminanceTable[]: Gamma = 2.2, Output Range = 230
const uint8_t coolWhiteLuminanceTable[] = {
    0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3, 
    3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6, 
    6,   7,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,  11,  11, 
   11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  17,  18, 
   18,  19,  19,  20,  20,  21,  22,  22,  23,  23,  24,  24,  25,  25,  26,  27, 
   27,  28,  29,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  37, 
   38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,  48,  48,  49,  50, 
   51,  52,  53,  54,  55,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65, 
   66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  79,  80,  81,  82, 
   83,  84,  85,  86,  88,  89,  90,  91,  92,  94,  95,  96,  97,  98, 100, 101, 
  102, 104, 105, 106, 107, 109, 110, 111, 113, 114, 115, 117, 118, 119, 121, 122, 
  124, 125, 127, 128, 129, 131, 132, 134, 135, 137, 138, 140, 141, 143, 144, 146, 
  147, 149, 151, 152, 154, 155, 157, 159, 160, 162, 163, 165, 167, 168, 170, 172, 
  173, 175, 177, 179, 180, 182, 184, 186, 187, 189, 191, 193, 194, 196, 198, 200, 
  202, 204, 205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 225, 227, 229, 230
};

void updateLights()
{
    uint8_t warmWhiteLEDBrightness, coolWhiteLEDBrightness;

    // Process the value of light outputs
    // We use analogWrite() as all output pins support PWM
    if (checkFunc(cvData[cvLightFctCtrl].value))
    {
        if(!cvData[cvLightTest].value)
        {
            // Check if we have to use brightness and CCT parameters set 1 or 2
            if (cvData[cvLightFctCtrl2].value != 255 && checkFunc(cvData[cvLightFctCtrl2].value))
            {
                // Use Set 2
                // Note: C always performs arithmetic operations in the size of the largest involved datatype
                // Here we cast the operands to uint16_t
                warmWhiteLEDBrightness = ((uint16_t)cvData[cvLightBrightness2].value * (255 - (uint16_t)cvData[cvLightColorTemperature2].value)) / 256;
                coolWhiteLEDBrightness = ((uint16_t)cvData[cvLightBrightness2].value * (uint16_t)cvData[cvLightColorTemperature2].value) / 256;
            }
            else
            {
                // Use Set 1
                warmWhiteLEDBrightness = ((uint16_t)cvData[cvLightBrightness].value * (255 - (uint16_t)cvData[cvLightColorTemperature].value)) / 256;
                coolWhiteLEDBrightness = ((uint16_t)cvData[cvLightBrightness].value * (uint16_t)cvData[cvLightColorTemperature].value) / 256;
            }
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
            analogWrite(pinLight[warmWhiteLight], cvData[cvLightBrightness].value);
            analogWrite(pinLight[coolWhiteLight], cvData[cvLightColorTemperature].value);
        }
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

    digitalWrite(pinLight[warmWhiteLight], HIGH);
    digitalWrite(pinLight[coolWhiteLight], HIGH);
    delay(6);
    analogWrite(pinLight[warmWhiteLight], 0);
    analogWrite(pinLight[coolWhiteLight], 0);
}

void setup()
{
    // Set light pins to outputs
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
    {
        analogWrite(pinLight[lightNr], 0);
        pinMode(pinLight[lightNr], OUTPUT);
    }

#ifdef DEBUG
    // Serial TX used for debugging messages. RX not used (not connected)
    // Two mapping options for Serial:
    //   First set (default): TX, RX, XCK, XDIR on PB2, PB3, PB1, PB0
    //   Second set :                              PA1, PA2, PA3, PA4
    Serial.swap(); // Use the second set of serial pins. TX is on now PA1
    Serial.begin(115200);
    Serial.println();
    Serial.println("-- Starting tiny DCC interior light decoder --");
#endif

    readFuncsToCache();
    readCVsToCache();

    // Initialize the NmraDcc library
    // void NmraDcc::pin (uint8_t ExtIntPinNum, uint8_t EnablePullup)
    // void NmraDcc::init (uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV)
    // COMMIT_NUMBER is defined in version.h
    dcc.pin(pinDCCInput, false);
    dcc.init(MAN_ID_DIY, COMMIT_COUNT, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

    // Commented out as not necessary. Uncomment for debugging purposes only. notifyCVResetFactoryDefault() is
    // automatically called at the very first call (i.e. unprogrammed EEPROM) of
    // NmraDcc::init() when FLAGS_AUTO_FACTORY_DEFAULT is set
    // notifyCVResetFactoryDefault();

    updateLights();

#ifdef DEBUG
    Serial.print("DCC Address: ");
    Serial.println(dcc.getAddr());
#endif

}

#ifdef DEBUG
uint32_t stillAliveCounterLow = 200000;
uint32_t stillAliveCounterHigh = 0;
#endif

void loop()
{
#ifdef DEBUG
    if (stillAliveCounterLow == 200000)
    {
        stillAliveCounterLow = 0;
        Serial.print("still alive ");
        Serial.println(stillAliveCounterHigh);
        stillAliveCounterHigh++;
    }
    stillAliveCounterLow++;
#endif

    // Process DCC packets
    dcc.process();

    // Handle resetting CVs to Factory Defaults
    if (factoryDefaultCVIndex && dcc.isSetCVReady())
    {
        factoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
        if (cvData[factoryDefaultCVIndex].applyDefault)
            dcc.setCV(cvData[factoryDefaultCVIndex].cvNr, cvData[factoryDefaultCVIndex].defaultValue);
    }
}