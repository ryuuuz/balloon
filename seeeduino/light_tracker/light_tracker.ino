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

#define _REGCODE_US915 3

// SX1262 has the following connections:
#define nssPin 8
#define rstPin 9
#define dio1Pin 3
#define busyPin 2

#define txPin 10
#define rxPin 12

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes.

static const u1_t PROGMEM DEVEUI[8] = {0x6C, 0x15, 0xB4, 0x03, 0x22, 0xF6, 0xEE, 0x06}; // helium or ttn

// This EUI must be in little-endian format, so least-significant-byte (lsb)
// first. When copying an EUI from Helium Console or ttnctl output, this means to reverse the bytes.

static const u1_t PROGMEM APPEUI[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07}; // helium or ttn

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In practice, a key taken from Helium Console or ttnctl can be copied as-is.

static const u1_t PROGMEM APPKEY[16] = {0x76, 0xE2, 0x0D, 0x5D, 0xED, 0x02, 0x06, 0xF1, 0xE2, 0x7D, 0xB5, 0xE4, 0x9E, 0xA1, 0xAB, 0xB9}; // helium or ttn

boolean OTAAJoinStatus = false; // do not change this.
int channelNoFor2ndSubBand = 0; // do not change this. Used for US915 and AU915 / TTN and Helium
uint32_t last_packet = 0;       // do not change this. Timestamp of last packet sent.

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

// #define Serial Serial1
// #define USE_SERIAL

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
    Serial.println();

    Serial.print(F("Time: "));         // Print the time
    uint8_t hms = ubxDataStruct->hour; // Print the hours
    if (hms < 10)
        Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->min; // Print the minutes
    if (hms < 10)
        Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->sec; // Print the seconds
    if (hms < 10)
        Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    uint32_t millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100)
        Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10)
        Serial.print(F("0"));
    Serial.print(millisecs);

    int32_t latitude = ubxDataStruct->lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    int32_t longitude = ubxDataStruct->lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
}

void processLMICEvents();
void initAndJoinWithLMIC();
void initAndJoinWithAT();
void sendDataWithLMIC(const char *data);
void sendDataWithAT(const char *data);

// the setup function runs once when you press reset or power the board
void setup()
{
    // 调用自定义时钟初始化函数
    initClock();

    Serial.begin(115200, SERIAL_8N1);
    Serial.println("Serial Ready!");

    #ifndef USE_SERIAL
    Serial1.begin(9600, SERIAL_8N1);
    Serial.println("Serial1 Ready!");
    #endif

    // Initialize the LED pin
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x47)
    while (pressureSensor.beginI2C(i2cAddress) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP581 connected!");

    // myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    if (myGNSS.begin() == false) // Connect to the u-blox module using Wire port
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }

    // Read the module info
    if (myGNSS.getModuleInfo()) // This line is optional. getModuleName() etc. will read the info if required
    {
        Serial.print(F("The GNSS module is: "));
        Serial.println(myGNSS.getModuleName());

        Serial.print(F("The firmware type is: "));
        Serial.println(myGNSS.getFirmwareType());

        Serial.print(F("The firmware version is: "));
        Serial.print(myGNSS.getFirmwareVersionHigh());
        Serial.print(F("."));
        Serial.println(myGNSS.getFirmwareVersionLow());

        Serial.print(F("The protocol version is: "));
        Serial.print(myGNSS.getProtocolVersionHigh());
        Serial.print(F("."));
        Serial.println(myGNSS.getProtocolVersionLow());
    }

    // initGNSS();

    // Example: Initialize and join with LMIC
    // initAndJoinWithLMIC();

    // Example: Initialize and join with AT commands
    initAndJoinWithAT();

    // // 初始化USB串口
    // Serial.begin(115200);
    // while (!Serial) {
    //   // 等待串口连接
    // }
    // Serial.println("USB CDC Ready!");
}

void fetchBMP581Data();
void fetchGNSSData();

// the loop function runs over and over again forever
void loop()
{
    // fetchBMP581Data();
    // fetchGNSSData();

    // processLMICEvents(); // Process the LMIC event queue
    // sendDataWithLMIC("Hello, LMIC!");

    sendDataWithAT("Hello, AT!");

    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(3000);                     // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(3000);                     // wait for a second
}

void initGNSS()
{
    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)

    myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).

    // Let's say that we want our 1 pulse every 3 seconds to be as accurate as possible. So, let's tell the module
    // to generate no signal while it is _locking_ to GNSS time. We want the signal to start only when the module is
    // _locked_ to GNSS time.
    myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_TP1, 0); // Set the period to zero
    myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_TP1, 0);    // Set the pulse length to zero

    // When the module is _locked_ to GNSS time, make it generate a 1 second pulse every 3 seconds
    myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_LOCK_TP1, 3000000); // Set the period to 3,000,000 us
    myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_LOCK_TP1, 1000000);    // Set the pulse length to 1,000,000 us

    myGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1);          // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
    myGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1);   // Tell the module to use PERIOD while locking and PERIOD_LOCK when locked to GNSS time
    myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 0);        // Tell the module that we want to set the period (not the frequency). PERIOD = 0. FREQ = 1.
    myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 1); // Tell the module to set the pulse length (not the pulse ratio / duty). RATIO = 0. LENGTH = 1.
    myGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1);          // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.

    // Now set the time pulse parameters
    if (myGNSS.sendCfgValset() == false)
    {
        Serial.println(F("VALSET failed!"));
    }
    else
    {
        Serial.println(F("VALSET Success!"));
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
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
        Serial.print("Temperature (C): ");
        Serial.print(data.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure (Pa): ");
        Serial.println(data.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
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
        Serial.print(F("Fix type: "));
        Serial.println(fix_type);

        uint8_t siv = myGNSS.getSIV();
        Serial.print(F(" SIV: "));
        Serial.println(siv);

        int32_t latitude = myGNSS.getLatitude();
        Serial.print(F("Lat: "));
        Serial.print(latitude);

        int32_t longitude = myGNSS.getLongitude();
        Serial.print(F(" Long: "));
        Serial.print(longitude);
        Serial.print(F(" (degrees * 10^-7)"));

        int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
        Serial.print(F(" Alt: "));
        Serial.print(altitude);
        Serial.print(F(" (mm)"));

        Serial.println();
        Serial.println();
    }
    else
    {
        Serial.println(F("No new data."));
    }

    // myGNSS.checkUblox();     // Check for the arrival of new data and process it.
    // myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
}

u1_t os_getRegion(void) { return _REGCODE_US915; }
void os_getJoinEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getNwkKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

void onLmicEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        packetQueued = false;
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_SCAN_FOUND:
        Serial.println(F("EV_SCAN_FOUND"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXDONE:
        Serial.println(F("EV_TXDONE"));
        break;
    case EV_DATARATE:
        Serial.println(F("EV_DATARATE"));
        break;
    case EV_START_SCAN:
        Serial.println(F("EV_START_SCAN"));
        break;
    case EV_ADR_BACKOFF:
        Serial.println(F("EV_ADR_BACKOFF"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println(ev);
        break;
    }
}

void processLMICEvents()
{
    os_runstep();
}

void initAndJoinWithLMIC()
{
    Serial.println(F("Initializing E22-900M22S (LMIC)..."));

    os_init(nullptr); // LMIC initialization
    LMIC_reset();

    // Start OTAA join
    LMIC_startJoining();
    Serial.println(F("Starting OTAA join..."));

    LMIC_setDrTxpow(0, KEEP_TXPOWADJ);
    LMIC_selectChannel(7);
    LMIC_setAdrMode(false);

    // Wait for OTAA join to complete
    while (LMIC.opmode & OP_JOINING)
    {
        processLMICEvents();
    }

    OTAAJoinStatus = true;
    Serial.println(F("LoRaWAN OTAA Join successful!"));
}

void initAndJoinWithAT() {
    Serial.println(F("Initializing E78-915LN22S (AT Commands)..."));

    // LOW for low power mode, HIGH for normal mode
    pinMode(SETB_PIN, OUTPUT);
    digitalWrite(SETB_PIN, HIGH);
    delay(100);

    // Example AT command sequence for E78-915LN22S

    Serial1.println("AT+IREBOOT=0");
    delay(3000);

    Serial1.println("AT+CGMI?");
    delay(100);
    Serial1.println("AT+CGMM?");
    delay(100);
    Serial1.println("AT+CGMR?");
    delay(100);
    Serial1.println("AT+CGSN?");
    delay(100);
    Serial1.println("AT+CGBR?");
    delay(100);
    Serial1.println("AT+CFREQBANDMASK?");
    delay(100);
    Serial1.println("AT+CNWM?");
    delay(100);


    Serial1.println("AT+REGIONCFG=US915"); // Set region to US915
    delay(100);

    // Serial1.println("AT+REGION=1"); // Set region to US915
    // delay(100);

    Serial1.println("AT+CAPPKEY=493d6071e1c131c1850224aae96078fe"); // Set AppKey
    delay(100);

    Serial1.println("AT+CAPPEUI=229a2a57ea7bfdf7"); // Set AppEUI
    delay(100);

    Serial1.println("AT+CDEVEUI=5daa5fd696722092"); // Set DevEUI
    delay(100);

    Serial1.println("AT+CAPPKEY?"); // Check AppKey
    delay(100);

    Serial1.println("AT+CAPPEUI?"); // Check AppEUI
    delay(100);

    Serial1.println("AT+CDEVEUI?"); // Check DevEUI
    delay(100);

    // Serial1.println("AT+CULDLMODE=2");
    // delay(100);

    Serial1.println("AT+CCLASS=0"); // Set Class A
    delay(100);

    Serial1.println("AT+CJOINMODE=0"); // Set OTAA mode
    delay(100);

    // Serial1.println("AT+CSTATUS?"); // Check status
    // delay(100);

    Serial1.println("AT+CJOIN=1,0,8,8"); // Join
    delay(30 * 1000);

    String response = "";
    while (Serial1.available()) {
        response += (char)Serial1.read();
    }

    if (response.indexOf("+CJOIN:OK") != -1) {
        Serial.println(F("E78-915LN22S LoRaWAN OTAA Join successful!"));
    } else {
        Serial.println(F("E78-915LN22S OTAA Join failed!"));
    }
}

void sendDataWithLMIC(const char *data)
{
    LMIC_setDrTxpow(1, KEEP_TXPOWADJ);

    LMIC_selectChannel(channelNoFor2ndSubBand);

    ++channelNoFor2ndSubBand;
    if (channelNoFor2ndSubBand > 7)
    {
        channelNoFor2ndSubBand = 0;
    }
    Serial.print(F("Channel: "));
    Serial.println(channelNoFor2ndSubBand);

    LMIC_setAdrMode(false);
    LMIC_setLinkCheckMode(0);

    if ((LMIC.opmode & OP_TXRXPEND) != 0)
    {
        Serial.println(F("Transmission pending, cannot queue new data"));
    }
    else
    {
        LMIC_setTxData2(8, (uint8_t *)data, strlen(data), 0);
        Serial.println(F("Packet queued with LMIC"));
    }
}

void sendDataWithAT(const char* data) {
    // 确定原始数据长度（以字节为单位）
    int dataLength = strlen(data);

    // 如果数据长度为 0，发送空包
    if (dataLength == 0) {
        Serial1.println("AT+DTRX=0,0,0,0");
        delay(100);
        Serial.println(F("Sent empty data packet"));
        return;
    }

    // 将数据转换为16进制格式（2个字符表示1个字节）
    String hexPayload = "";
    for (int i = 0; i < dataLength; i++) {
        char hexByte[3];  // 每字节两位16进制，外加1个空字符
        sprintf(hexByte, "%02X", (unsigned char)data[i]);
        hexPayload += hexByte;
    }

    // 检查长度是否合法（LoRaWAN规范的最大长度）
    if (dataLength > 255) {  // 根据具体设备要求调整最大值
        Serial.println(F("Error: Data length exceeds maximum allowed limit"));
        return;
    }

    // 构建 AT 指令
    Serial1.print("AT+DTRX=0,0,");   // 确认模式0，不重传
    Serial1.print(dataLength);       // 数据长度
    Serial1.print(",");              // 逗号分隔符
    Serial1.println(hexPayload);     // 发送16进制负载数据

    // 等待响应
    delay(100);

    // 接收设备响应
    String response = "";
    while (Serial1.available()) {
        response += (char)Serial1.read();
    }

    // 检查响应内容
    if (response.indexOf("OK+SEND") != -1) {
        Serial.println(F("Packet sent successfully with AT commands"));
    } else if (response.indexOf("ERR") != -1) {
        Serial.println(F("Error: Sending failed. Response: "));
        Serial.println(response);
    } else {
        Serial.println(F("Error: No valid response from device"));
    }
}

