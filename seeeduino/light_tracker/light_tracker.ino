/*
  Light Tracker

  Feature
  - custom 32.768kHz Crystal

  by Yuzhou Ren

  https://github.com/ryuuuz/balloon/tree/main/seeeduino
*/

#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#include "SparkFun_u-blox_GNSS_v3.h" //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

#include <basicmac.h>
#include <hal/hal.h>

#include <CayenneLPP.h>

#define _REGCODE_US915 3

// SX1262 has the following connections:
#define nssPin 8
#define rstPin 9
#define dio1Pin 3
#define busyPin 2

#define BattPin A5
#define GpsPwr 12
#define GpsON digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes.

static const u1_t PROGMEM DEVEUI[8] = {0x06, 0xef, 0xd3, 0x85, 0xcb, 0xf6, 0xc1, 0x2b}; // helium or ttn

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes.

static const u1_t PROGMEM APPEUI[8] = {0x53, 0xb8, 0x67, 0xb0, 0xd3, 0xae, 0xe1, 0x38}; // helium or ttn

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In practice, a key taken from Helium Console or ttnctl can be copied as-is.

static const u1_t PROGMEM APPKEY[16] = {0x74, 0x5b, 0x9b, 0x1a, 0xce, 0xe7, 0xb8, 0xa2, 0xa9, 0x8f, 0x48, 0x62, 0x7a, 0x16, 0x49, 0x88}; // helium or ttn

boolean OTAAJoinStatus = false; // do not change this.
int channelNoFor2ndSubBand = 0; // do not change this. Used for US915 and AU915 / TTN and Helium
uint32_t last_packet = 0;       // do not change this. Timestamp of last packet sent.


// ABP
// LoRaWAN NwkSKey, network session key (Big Endian)
static const PROGMEM u1_t NWKSKEY[16] = { 
    0x87, 0x86, 0xD5, 0x5B, 0x25, 0x1B, 0xDD, 0x24, 
    0x81, 0x4A, 0xDE, 0x81, 0x14, 0xA2, 0x4E, 0x0C 
};

// LoRaWAN AppSKey, application session key (Big Endian)
static const u1_t PROGMEM APPSKEY[16] = { 
    0x26, 0xF1, 0xB3, 0x9E, 0x3E, 0x65, 0xB2, 0x6A, 
    0xCF, 0xA3, 0x8E, 0xB3, 0xD2, 0x7B, 0x50, 0xE9 
};

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x019a39b7 ; // <-- Change this address for every node!


uint8_t measurementSystem = 0; // 0 for metric (meters, km, Celcius, etc.), 1 for imperial (feet, mile, Fahrenheit,etc.)

// try to keep telemetry size smaller than 51 bytes if possible. Default telemetry size is 37 bytes.
CayenneLPP telemetry(37);

// pinmap for SX1262 LoRa module
const lmic_pinmap lmic_pins = {
    .nss = nssPin,
    .tx = LMIC_UNUSED_PIN,
    .rx = LMIC_UNUSED_PIN,
    .rst = rstPin,
    .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ dio1Pin, /* DIO2 */ LMIC_UNUSED_PIN},
    .busy = busyPin,
    .tcxo = LMIC_CONTROLLED_BY_DIO3,
};

//********************************* Misc Settings ******************************
int txCount = 1;
float voltage = 0;
boolean packetQueued = false;

#define SETB_PIN 43

#include "variant.h"

#undef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR Serial

#define USB_MANUFACTURER "Your Company"
#define USB_PRODUCT "ATSAMD21 CDC Device"

// 自定义时钟初始化函数
void initClock()
{
    // 配置外部晶振（XOSC32K）
    SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP(0x6) | // 启动时间
                           SYSCTRL_XOSC32K_XTALEN |       // 启用晶振
                           SYSCTRL_XOSC32K_EN32K |        // 输出32.768kHz信号
                           SYSCTRL_XOSC32K_ENABLE;        // 启用 XOSC32K
    while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY)
        ; // 等待晶振稳定

    // 配置 GCLK1 使用 XOSC32K 作为时钟源
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) |         // GCLK1
                       GCLK_GENDIV_DIV(1);         // 分频系数1
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) |       // GCLK1
                        GCLK_GENCTRL_SRC_XOSC32K | // 时钟源为外部晶振
                        GCLK_GENCTRL_GENEN;        // 启用 GCLK1
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // 等待同步完成

    // 将 GCLK1 分配到 DFLL48M（主系统时钟）
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_DFLL48) |
                        GCLK_CLKCTRL_GEN(1) | // GCLK1 -> DFLL48M
                        GCLK_CLKCTRL_CLKEN;   // 启用 DFLL48M
    while (GCLK->STATUS.bit.SYNCBUSY)
        ; // 等待同步完成
}

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
// uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

void initGNSS();

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    SerialUSB.println();

    SerialUSB.print(F("Time: "));         // Print the time
    uint8_t hms = ubxDataStruct->hour; // Print the hours
    if (hms < 10)
        SerialUSB.print(F("0")); // Print a leading zero if required
    SerialUSB.print(hms);
    SerialUSB.print(F(":"));
    hms = ubxDataStruct->min; // Print the minutes
    if (hms < 10)
        SerialUSB.print(F("0")); // Print a leading zero if required
    SerialUSB.print(hms);
    SerialUSB.print(F(":"));
    hms = ubxDataStruct->sec; // Print the seconds
    if (hms < 10)
        SerialUSB.print(F("0")); // Print a leading zero if required
    SerialUSB.print(hms);
    SerialUSB.print(F("."));
    uint32_t millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100)
        SerialUSB.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10)
        SerialUSB.print(F("0"));
    SerialUSB.print(millisecs);

    int32_t latitude = ubxDataStruct->lat; // Print the latitude
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(latitude);

    int32_t longitude = ubxDataStruct->lon; // Print the longitude
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(longitude);
    SerialUSB.print(F(" (degrees * 10^-7)"));

    int32_t altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    SerialUSB.print(F(" Height above MSL: "));
    SerialUSB.print(altitude);
    SerialUSB.println(F(" (mm)"));
}

void processLMICEvents();
void initAndJoinWithLMIC();
void sendDataWithLMIC(const char *data);

void initThroughABP();


// the setup function runs once when you press reset or power the board
void setup()
{
    // 调用自定义时钟初始化函数
    // initClock();

    delay(5000); // do not change this

    SerialUSB.begin(115200, SERIAL_8N1);
    SerialUSB.println("Serial Ready!");

    // Initialize the LED pin
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize the I2C library
    Wire.begin();

    // Initialize sensor connection with retry mechanism
    int8_t err = BMP5_OK;
    unsigned long retryInterval = 1000;  // Retry interval in milliseconds
    int retryCount = 10;  // Number of retries before giving up

    // Try initializing the sensor
    while (retryCount > 0) {
        err = pressureSensor.beginI2C(i2cAddress);
        
        if (err == BMP5_OK) {
            break;  // Successfully connected, exit loop
        }

        // Not connected, inform user and retry
        SerialUSB.println("Error: BMP581 not connected, check wiring and I2C address!");
        SerialUSB.print("Error code: ");
        SerialUSB.println(err);
        
        retryCount--;
        if (retryCount > 0) {
            SerialUSB.print("Retrying in ");
            SerialUSB.print(retryInterval / 1000);
            SerialUSB.println(" seconds...");
            delay(retryInterval);
        }
    }

    if (err != BMP5_OK) {
        SerialUSB.println("Failed to initialize BMP581 sensor after multiple attempts.");
    }

    SerialUSB.println("BMP581 connected!");

    // myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    pinMode(GpsPwr, OUTPUT);
    GpsON;
    delay(500);
    
    if (myGNSS.begin() == false) // Connect to the u-blox module using Wire port
    {
        SerialUSB.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }

    // Read the module info
    if (myGNSS.getModuleInfo()) // This line is optional. getModuleName() etc. will read the info if required
    {
        SerialUSB.print(F("The GNSS module is: "));
        SerialUSB.println(myGNSS.getModuleName());

        SerialUSB.print(F("The firmware type is: "));
        SerialUSB.println(myGNSS.getFirmwareType());

        SerialUSB.print(F("The firmware version is: "));
        SerialUSB.print(myGNSS.getFirmwareVersionHigh());
        SerialUSB.print(F("."));
        SerialUSB.println(myGNSS.getFirmwareVersionLow());

        SerialUSB.print(F("The protocol version is: "));
        SerialUSB.print(myGNSS.getProtocolVersionHigh());
        SerialUSB.print(F("."));
        SerialUSB.println(myGNSS.getProtocolVersionLow());
    }

    initGNSS();

    // Initialize and join with LMIC
    // initAndJoinWithLMIC();

    // Initialize through ABP
    initThroughABP();
}

void fetchBMP581Data();
void fetchGNSSData();

// the loop function runs over and over again forever
void loop()
{
    fetchBMP581Data();
    fetchGNSSData();

    processLMICEvents(); // Process the LMIC event queue
    updateTelemetry();
    sendLoRaWANPacket();

    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(1000);                     // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(1000);                     // wait for a second
}

void setupUBloxDynamicModel()
{
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    // DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters.
    if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
        SerialUSB.println(F("***!!! Warning: setDynamicModel failed !!!***"));
    }
    else
    {
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGNSS.getDynamicModel());
    }
}

void initGNSS()
{
    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)

    myGNSS.setNavigationFrequency(2);            // Set output to 2 times a second. Max is 10
    byte rate = myGNSS.getNavigationFrequency(); // Get the update rate of this module
    SerialUSB.print("Current update rate for GPS: ");
    SerialUSB.println(rate);

    myGNSS.saveConfiguration(); // Save the current settings to flash and BBR

    setupUBloxDynamicModel();
}

void fetchBMP581Data()
{
    // Get measurements from the sensor
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if (err == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        SerialUSB.print("Temperature (C): ");
        SerialUSB.print(data.temperature);
        SerialUSB.print("\t\t");
        SerialUSB.print("Pressure (Pa): ");
        SerialUSB.println(data.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        SerialUSB.print("Error getting data from sensor! Error code: ");
        SerialUSB.println(err);
    }
}

void fetchGNSSData()
{
    // Request (poll) the position, velocity and time (PVT) information.
    // The module only responds when a new position is available. Default is once per second.
    // getPVT() returns true when new data is received.
    if (myGNSS.getPVT() == true)
    {
        uint8_t fix_type = myGNSS.getFixType();
        SerialUSB.print(F("Fix type: "));
        SerialUSB.print(fix_type);

        uint8_t siv = myGNSS.getSIV();
        SerialUSB.print(F(" SIV: "));
        SerialUSB.print(siv);

        int32_t latitude = myGNSS.getLatitude();
        SerialUSB.print(F(" Lat: "));
        SerialUSB.print(latitude);

        int32_t longitude = myGNSS.getLongitude();
        SerialUSB.print(F(" Long: "));
        SerialUSB.print(longitude);
        SerialUSB.print(F(" (degrees * 10^-7)"));

        int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
        SerialUSB.print(F(" Alt: "));
        SerialUSB.print(altitude);
        SerialUSB.print(F(" (mm)"));

        SerialUSB.println();
    }
    else
    {
        SerialUSB.println(F("No new data."));
    }

    // myGNSS.checkUblox();     // Check for the arrival of new data and process it.
    // myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
}

float readBatt()
{

    float R1 = 560000.0; // 560K
    float R2 = 100000.0; // 100K
    float value = 0.0f;

    do
    {
        value = analogRead(BattPin);
        value += analogRead(BattPin);
        value += analogRead(BattPin);
        value = value / 3.0f;
        value = (value * 3.3) / 1024.0f;
        value = value / (R2 / (R1 + R2));
    } while (value > 20.0);

    return value;
}

void updateTelemetry()
{
    // Get measurements from the sensor
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    voltage = readBatt();

    float tempAltitudeLong = 0;  // meters or feet
    float tempAltitudeShort = 0; // km or miles
    float tempSpeed = 0;         // km or miles
    float tempTemperature = 0;   // Celcius or Fahrenheit

    if (measurementSystem == 0)
    { // Metric

        tempAltitudeLong = myGNSS.getAltitude() / 1000.f; // meters
        tempAltitudeShort = tempAltitudeLong / 1000.f;   // kilometers
        tempSpeed = myGNSS.getGroundSpeed() * 0.0036f;    // km/hour
        tempTemperature = data.temperature;             // Celsius
    }
    else
    { // Imperial

        tempAltitudeLong = (myGNSS.getAltitude() * 3.2808399) / 1000.f; // feet
        tempAltitudeShort = tempAltitudeLong / 5280.f;                 // miles
        tempSpeed = myGNSS.getGroundSpeed() * 0.00223694f;              // mile/hour
        tempTemperature = (data.temperature * 1.8f) + 32;             // Fahrenheit
    }

    // latitude,longtitude,altitude,speed,course,sattelite,battery,temp,pressure

    telemetry.reset();                                                                                          // clear the buffer
    telemetry.addGPS(1, myGNSS.getLatitude() / 10000000.f, myGNSS.getLongitude() / 10000000.f, tempAltitudeLong); // channel 3, coordinates and altitude (meters or feet)
    telemetry.addTemperature(2, tempTemperature);                                                               // Celcius or Fahrenheit
    telemetry.addAnalogInput(3, voltage);                                                                       // Battery/Supercaps voltage
    telemetry.addDigitalInput(4, myGNSS.getSIV());                                                               // GPS sattelites in view
    telemetry.addAnalogInput(5, tempSpeed);                                                                     // km/h or mile/h
    telemetry.addDigitalInput(6, myGNSS.getHeading() / 100000);                                                  // course in degrees
    telemetry.addBarometricPressure(7, data.pressure / 100.f);                                                 // pressure
    telemetry.addAnalogInput(8, tempAltitudeShort);                                                             // kilometers or miles
}

u1_t os_getRegion(void) { return _REGCODE_US915; }
void os_getJoinEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getNwkKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

void onLmicEvent(ev_t ev)
{
    SerialUSB.print(os_getTime());
    SerialUSB.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        SerialUSB.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        SerialUSB.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        SerialUSB.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        SerialUSB.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        SerialUSB.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        SerialUSB.println(F("EV_JOINED"));

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        SerialUSB.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        SerialUSB.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        SerialUSB.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        SerialUSB.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        packetQueued = false;
        if (LMIC.txrxFlags & TXRX_ACK)
            SerialUSB.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            SerialUSB.print(F("Received "));
            SerialUSB.print(LMIC.dataLen);
            SerialUSB.println(F(" bytes of payload"));
        }
        break;
    case EV_LOST_TSYNC:
        SerialUSB.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        SerialUSB.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        SerialUSB.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        SerialUSB.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        SerialUSB.println(F("EV_LINK_ALIVE"));
        break;
    case EV_SCAN_FOUND:
        SerialUSB.println(F("EV_SCAN_FOUND"));
        break;
    case EV_TXSTART:
        SerialUSB.println(F("EV_TXSTART"));
        break;
    case EV_TXDONE:
        SerialUSB.println(F("EV_TXDONE"));
        break;
    case EV_DATARATE:
        SerialUSB.println(F("EV_DATARATE"));
        break;
    case EV_START_SCAN:
        SerialUSB.println(F("EV_START_SCAN"));
        break;
    case EV_ADR_BACKOFF:
        SerialUSB.println(F("EV_ADR_BACKOFF"));
        break;

    default:
        SerialUSB.print(F("Unknown event: "));
        SerialUSB.println(ev);
        break;
    }
}

void processLMICEvents()
{
    os_runstep();
}

void initAndJoinWithLMIC()
{
    SerialUSB.println(F("Initializing E22-900M22S (LMIC)..."));

    os_init(nullptr); // LMIC initialization
    LMIC_reset();

    // Start OTAA join
    LMIC_startJoining();
    SerialUSB.println(F("Starting OTAA join..."));

    SerialUSB.println(F("Region US915"));
    // LMIC_setDrTxpow(0, KEEP_TXPOWADJ);
    // LMIC_selectChannel(7);

    // LMIC_setAdrMode(false);
    // LMIC_setLinkCheckMode(0);

    // Wait for OTAA join to complete
    while (LMIC.opmode & OP_JOINING)
    {
        processLMICEvents();
    }

    OTAAJoinStatus = true;
    SerialUSB.println(F("LoRaWAN OTAA Join successful!"));
}

void initThroughABP() {
    SerialUSB.println(F("Initializing E22-900M22S (LMIC) through ABP..."));

    os_init(nullptr); // LMIC initialization
    LMIC_reset();

    // Set the session keys
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Set the data rate and transmit power
    LMIC_setDrTxpow(0, KEEP_TXPOWADJ);

    // Disable ADR
    LMIC_setAdrMode(false);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    OTAAJoinStatus = true;
    SerialUSB.println(F("LoRaWAN ABP initialization successful!"));
}

// Telemetry size is very important, try to keep it lower than 51 bytes. Always lower is better.
void sendLoRaWANPacket()
{
    if (telemetry.getSize() < 54)
    {
        // DR1 (SF9 BW125kHz) max payload size is 53 bytes.
        LMIC_setDrTxpow(1, KEEP_TXPOWADJ);
    }
    else if (telemetry.getSize() < 126)
    {
        // DR2 (SF8 BW125kHz) max payload size is 125 bytes.
        LMIC_setDrTxpow(2, KEEP_TXPOWADJ);
    }
    else
    {
        // DR3 (SF7 BW125kHz) max payload size is 222 bytes.
        LMIC_setDrTxpow(3, KEEP_TXPOWADJ);
    }
    
    
    LMIC_selectChannel(channelNoFor2ndSubBand);

    ++channelNoFor2ndSubBand;
    if (channelNoFor2ndSubBand > 7)
    {
        channelNoFor2ndSubBand = 0;
    }

    SerialUSB.print(F("Channel: "));
    SerialUSB.println(channelNoFor2ndSubBand);

    LMIC_setAdrMode(false);
    LMIC_setLinkCheckMode(0);

    LMIC_setTxData2(1, telemetry.getBuffer(), telemetry.getSize(), 0);
    last_packet = millis();
    txCount++;
    packetQueued = true;
    SerialUSB.print(F("Packet queued..."));
}
