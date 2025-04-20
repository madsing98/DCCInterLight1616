/*************************************************************************************************************\
Mobile DCC decoder based on attiny1616 for Kato cars interior lights

Core / libraries
- megaTinyCore: https://github.com/SpenceKonde/megaTinyCore
- NmraDcc: https://github.com/mrrwa/NmraDcc

Hardware resources
- megaTinyCore uses TCA0 for PWM with AnalogWrite()
    - See page 182 of data sheet, 20.3.3.4.3 Single-Slope PWM Generation
    - megaTinyCore uses split-mode, to have up to six 8-bit PWM outputs (WO (Waveform Out) [0..5]).
    See 20.3.3.6 Split Mode - Two 8-Bit Timer/Counters
    - From the table below, without using multiplex signals, PWM could be (for Attiny1616-MNR VQFN 20-pin)
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
    - On the attiny1616, millis() and micros() use TCD0

- EEPROM
    - attiny 1616 EEPROM size is 256 bytes
    - NmraDcc uses the EEPROM to store CVs. CVs are stored at the location corresponding to the CV number
      (i.e., CV29 is stored at location 29)
    - We will also use location 255 to store the status of the functions (F0 to F4). The goal is to have the lights
      in the correct state at power on, before the decoder receives any DCC packet setting these functions

CV Map
CV1     Primary Address
CV7     Manufacturer Version Number
CV8     Manufacturer ID Number
CV29    Mode Control

CV50    Light Brightness (0..255)
CV51    Light Temperature (0..255)
            0 = warm white
          128 = natural white (default)
          255 = cool white
CV52    Light Function control
          0 = F0 (default)
          1 = F1
          2 = F2
          3 = F3
          4 = F4
\*************************************************************************************************************/

#include <Arduino.h>
#include <NmraDcc.h>
#include <EEPROM.h>

// Uncomment to send debugging messages to the serial line
#define DEBUG

// Versioning
const uint8_t versionIdMajor = 0;
const uint8_t versionIdMinor = 1;
const uint8_t versionId = versionIdMajor << 4 | versionIdMinor;

// Hardware pin definitions
const uint8_t numberOfLights = 2;
const pin_size_t pinLight[numberOfLights] = {PIN_PB1, PIN_PB0};
//                      Light # on the PCB      0        1
const uint8_t warmWhiteLight = 0;
const uint8_t coolWhiteLight = 1;
const pin_size_t pinDCCInput = PIN_PA2;
const pin_size_t pinACKOutput = PIN_PA3;

// Objects from NmraDcc
NmraDcc Dcc;

// Current value of loco functions
uint8_t currentFuncState = 0;

// fctsCache[] holds the current state (ON/OFF) of functions F0 to F4
const uint8_t numberOfFctsInCache = 5;
bool fctsCache[numberOfFctsInCache];
const uint16_t fctsEepromAddress = 255;

// CV number definitions
const uint8_t CV0Check = 0;
const uint8_t CV1PrimaryAddress = 1;
const uint8_t CV7ManufacturerVersionNumber = 7;
const uint8_t CV8ManufacturerIDNumber = 8;
const uint8_t CV29ModeControl = 29;

// CVs related to light outputs
const uint8_t CV50LightBrightness = 50;
const uint8_t CV51LightTemperature = 51;
const uint8_t CV52LightFctCtrl = 52; // CV with the highest number

// cvsCache[] stores the CVs (in RAM, for quickest access)
// The indexes of the array are the CV numbers
// cvsCache[cvNumber] = cvValue
const uint8_t numberOfCvsInCache = CV52LightFctCtrl + 1; // CV52LightFctCtrl is the CV with the highest number
uint8_t cvsCache[numberOfCvsInCache];

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

        {CV50LightBrightness, 50},
        {CV51LightTemperature, 128},
        {CV52LightFctCtrl, 0}};

void updateLights();

// This callback function is called when a CV Value changes so we can update cvsCache[]
void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef DEBUG
    Serial.print("notifyCVChange: CV: ");
    Serial.print(CV);
    Serial.print(" Value: ");
    Serial.println(Value);
#endif

    if (CV < numberOfCvsInCache)
        cvsCache[CV] = Value;

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

// Function called at setup time to load all CVs to the array cvsCache[] in memory
// Only the CVs used (i.e. listed in FactoryDefaultCVs) are read
void readCvsToCache()
{
    for (uint8_t i = 0; i < sizeof(FactoryDefaultCVs) / sizeof(CVPair); i++)
    {
        uint16_t cvNr = FactoryDefaultCVs[i].CV;
        cvsCache[cvNr] = Dcc.getCV(cvNr);
#ifdef DEBUG
        Serial.print("CV");
        Serial.print(cvNr);
        Serial.print("=");
        Serial.print(cvsCache[cvNr]);
        Serial.print("|");
#endif
    }
#ifdef DEBUG
    Serial.println();
#endif
}

// This callback function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
    // Check that the DCC packet is for functions 0 to 4 (the only functions we use)
    // Check that one of the functions has changed
    if (FuncGrp == FN_0_4 && currentFuncState != FuncState)
    {
#ifdef DEBUG
        Serial.print("Function Group: ");
        Serial.print(FuncGrp);
        Serial.print("|State = 0b");
        Serial.println(FuncState, BIN);
#endif
        currentFuncState = FuncState;
        fctsCache[0] = (bool)(FuncState & FN_BIT_00);
        fctsCache[1] = (bool)(FuncState & FN_BIT_01);
        fctsCache[2] = (bool)(FuncState & FN_BIT_02);
        fctsCache[3] = (bool)(FuncState & FN_BIT_03);
        fctsCache[4] = (bool)(FuncState & FN_BIT_04);
        updateLights();
        EEPROM.write(fctsEepromAddress, FuncState);
    }
}

void readFctsToCache()
{
    uint8_t FuncState = EEPROM.read(fctsEepromAddress);
    currentFuncState = FuncState;
    fctsCache[0] = (bool)(FuncState & FN_BIT_00);
    fctsCache[1] = (bool)(FuncState & FN_BIT_01);
    fctsCache[2] = (bool)(FuncState & FN_BIT_02);
    fctsCache[3] = (bool)(FuncState & FN_BIT_03);
    fctsCache[4] = (bool)(FuncState & FN_BIT_04);
}

// Gamma table
const uint8_t gamma[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
    2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
    10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
    17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
    25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
    37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
    51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
    69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
    90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
    115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
    144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
    215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};

void updateLights()
{
    uint16_t warmWhiteLEDBrightness, coolWhiteLEDBrightness;

    // Process the value of light outputs
    // We use analogWrite() as all output pins support PWM
    if (fctsCache[cvsCache[CV52LightFctCtrl]])
    {
        warmWhiteLEDBrightness = (cvsCache[CV50LightBrightness] * (255 - cvsCache[CV51LightTemperature])) >> 8;
        coolWhiteLEDBrightness = (cvsCache[CV50LightBrightness] * cvsCache[CV51LightTemperature]) >> 8;
        analogWrite(pinLight[warmWhiteLight], gamma[warmWhiteLEDBrightness]);
        analogWrite(pinLight[coolWhiteLight], gamma[coolWhiteLEDBrightness]);
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

    digitalWrite(pinACKOutput, HIGH);
    delay(8);
    digitalWrite(pinACKOutput, LOW);
}

void setup()
{
    // Set light pins and DCC ACK pin to outputs
    for (uint8_t lightNr = 0; lightNr < numberOfLights; lightNr++)
    {
        analogWrite(pinLight[lightNr], 0);
        pinMode(pinLight[lightNr], OUTPUT);
    }

    digitalWrite(pinACKOutput, 0);
    pinMode(pinACKOutput, OUTPUT);

#ifdef DEBUG
    // Serial TX used for debugging messages
    // Two mapping options for Serial are PB2, PB3, PB1, PB0 (default) and PA1, PA2, PA3, PA4 for TX, RX, XCK, XDIR.
    Serial.swap(); // Use the second set of serial pins. TX is on PA1
    Serial.begin(115200);
    Serial.println();
    Serial.println("-- Starting tiny DCC decoder --");
#endif

    readFctsToCache();

    // Initialize the NmraDcc library
    // void NmraDcc::pin (uint8_t ExtIntPinNum, uint8_t EnablePullup)
    // void NmraDcc::init (uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV)
    Dcc.pin(pinDCCInput, false);
    Dcc.init(MAN_ID_DIY, versionId, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

    // Commented out as not necessary with Attiny
    // notifyCVResetFactoryDefault() is automatically called
    // at very first call (i.e. unprogrammed EEPROM) of NmraDcc::init() with FLAGS_AUTO_FACTORY_DEFAULT set
    // notifyCVResetFactoryDefault();

    readCvsToCache();
}

#ifdef DEBUG
uint16_t loopCounterLow = 20000;
uint16_t loopCounterHigh = 0;
#endif

void loop()
{
#ifdef DEBUG
    if (loopCounterLow == 20000)
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