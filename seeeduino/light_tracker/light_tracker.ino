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

#include "variant.h"

#undef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR SerialUSB

#define USB_MANUFACTURER "Your Company"
#define USB_PRODUCT "ATSAMD21 CDC Device"

#define Serial Serial1

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

// the setup function runs once when you press reset or power the board
void setup()
{
    // 调用自定义时钟初始化函数
    initClock();

    // initialize digital pin LED_BUILTIN as an output.
    Serial.begin(115200, SERIAL_8N1);
    Serial.println("Serial1 Ready!");

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

    // myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)

    // myGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).

    // // Let's say that we want our 1 pulse every 3 seconds to be as accurate as possible. So, let's tell the module
    // // to generate no signal while it is _locking_ to GNSS time. We want the signal to start only when the module is
    // // _locked_ to GNSS time.
    // myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_TP1, 0); // Set the period to zero
    // myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_TP1, 0);    // Set the pulse length to zero

    // // When the module is _locked_ to GNSS time, make it generate a 1 second pulse every 3 seconds
    // myGNSS.addCfgValset(UBLOX_CFG_TP_PERIOD_LOCK_TP1, 3000000); // Set the period to 3,000,000 us
    // myGNSS.addCfgValset(UBLOX_CFG_TP_LEN_LOCK_TP1, 1000000);    // Set the pulse length to 1,000,000 us

    // myGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1);          // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
    // myGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1);   // Tell the module to use PERIOD while locking and PERIOD_LOCK when locked to GNSS time
    // myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 0);        // Tell the module that we want to set the period (not the frequency). PERIOD = 0. FREQ = 1.
    // myGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 1); // Tell the module to set the pulse length (not the pulse ratio / duty). RATIO = 0. LENGTH = 1.
    // myGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1);          // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.

    // // Now set the time pulse parameters
    // if (myGNSS.sendCfgValset() == false)
    // {
    //     Serial.println(F("VALSET failed!"));
    // }
    // else
    // {
    //     Serial.println(F("VALSET Success!"));
    // }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)

    // // 初始化USB串口
    // SerialUSB.begin(115200);
    // while (!SerialUSB) {
    //   // 等待串口连接
    // }
    // SerialUSB.println("USB CDC Ready!");
}

// the loop function runs over and over again forever
void loop()
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

    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(1000);                     // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(1000);                     // wait for a second
}
