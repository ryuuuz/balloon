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
#include <LightTrackerGeofence.h>

#include <CayenneLPP.h>

// SX1262 has the following connections:
#define nssPin 8
#define rstPin 9
#define dio1Pin 3
#define busyPin 2

#define BattPin A5
#define GpsPwr 12
#define GpsON digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);

#define DEVMODE // Development mode. Uncomment to enable for debugging.

#ifdef DEVMODE
boolean ignoreFix = true;
#else
boolean ignoreFix = false;
#endif

int channelNoFor2ndSubBand = 0; // do not change this. Used for US915 and AU915 / TTN and Helium
uint32_t last_packet = 0;       // do not change this. Timestamp of last packet sent.

static const u1_t PROGMEM DEVEUI[8] = {0x06, 0xef, 0xd3, 0x85, 0xcb, 0xf6, 0xc1, 0x2b}; // helium or ttn

static const u1_t PROGMEM APPEUI[8] = {0x53, 0xb8, 0x67, 0xb0, 0xd3, 0xae, 0xe1, 0x38}; // helium or ttn


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

boolean ABPInitStatus = false; // do not change this.


uint8_t measurementSystem = 0; // 0 for metric (meters, km, Celcius, etc.), 1 for imperial (feet, mile, Fahrenheit,etc.)

//************************** LoRaWAN Settings ********************
const unsigned TX_INTERVAL = 10000;  // Schedule TX every this many miliseconds (might become longer due to duty cycle limitations).

// try to keep telemetry size smaller than 51 bytes if possible. Default telemetry size is 37 bytes.
CayenneLPP telemetry(51);

// string to send
String my_sentence = "hello from yuzhou";
uint8_t sentence_length = 0;             // do not change this.
int *indices;                            // do not change this.
bool sentenceSent = false;               // do not change this.
int letter_index_in_random_sequence = 0; // do not change this.

#define ENABLE_RANDOM_LETTER_SENDING true  // true to enable random letter sending, false to disable

// The LoRaWAN region to use, automatically selected based on your location. So GPS fix is necesarry
u1_t os_getRegion (void) { return Lorawan_Geofence_region_code; } //do not change this

// GEOFENCE 
uint8_t Lorawan_Geofence_no_tx  = 0; //do not change this
uint8_t Lorawan_Geofence_region_code = _REGCODE_UNDEF; //do not change this
uint8_t Lorawan_Geofence_special_region_code = _REGCODE_UNDEF; //do not change this

uint8_t lastLoRaWANRegion = _REGCODE_UNDEF; //do not change this

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

//************************** uBlox GPS  Settings ********************
boolean gpsFix=false; //do not change this.
boolean ublox_high_alt_mode_enabled = false; //do not change this.
boolean HardwareSetup=false; //do not change this.
//********************************* Power Settings ******************************
int   battWait=10;    //seconds sleep if super capacitors/batteries are below battMin (important if power source is solar panel) 
float battMin=2.5;    // min Volts to TX. (Works with 3.3V too but 3.5V is safer) 
float gpsMinVolt=3; //min Volts for GPS to wake up. (important if power source is solar panel) //do not change this
//********************************* Misc Settings ******************************
int txCount = 1;
float voltage = 0;
boolean packetQueued = false;

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

void initBMP581();

void processLMICEvents();
void initAndJoinWithLMIC();

void initThroughABP();


// the setup function runs once when you press reset or power the board
void setup()
{
    // 调用自定义时钟初始化函数
    // initClock();

    delay(5000); // do not change this

    pinMode(GpsPwr, OUTPUT);
    GpsOFF;
    
    SerialUSB.begin(115200, SERIAL_8N1);
    Serial.begin(115200);

    // Initialize the LED pin
    pinMode(LED_BUILTIN, OUTPUT);

    // initBMP581();


    // Wait up to 5 seconds for serial to be opened, to allow catching
    // startup messages on native USB boards (that do not reset when
    // serial is opened).
    unsigned long start = millis();
    while (millis() - start < 5000 && !SerialUSB);

    SerialUSB.println();
    SerialUSB.println(F("Starting"));
    SerialUSB.println();

    Serial.println();
    Serial.println(F("Starting"));
    Serial.println();

    sentence_length = my_sentence.length();
    indices = new int[sentence_length];

    randomSeed(analogRead(0)); // Seed the random number generator

    SerialUSB.print("Sentence: ");
    SerialUSB.println(my_sentence);
    SerialUSB.print("Sentence length: ");
    SerialUSB.println(sentence_length);
}

void fetchGNSSData();
void fetchBMP581Data();

// the loop function runs over and over again forever
void loop()
{
    voltage = readBatt();

    if (((voltage > battMin) && gpsFix) || ((voltage > gpsMinVolt) && !gpsFix)) {
        if (!HardwareSetup) {
            SerialUSB.println(F("GPS and BMP setup"));
            setup_GPS_BMP();
            SerialUSB.println(F("Searching for GPS fix..."));
            HardwareSetup = true;
        }

        if(gpsFix) {
            // Let LMIC handle LoRaWAN background tasks
            os_runstep(); 
        }
    
        if ((!packetQueued && (millis() - last_packet > TX_INTERVAL && !(LMIC.opmode & OP_TXRXPEND)) || !gpsFix)) {
            GpsON;
            SerialUSB.println(F("GPS ON"));
            delay(500);

            if(!ublox_high_alt_mode_enabled) {
                setupUBloxDynamicModel();
            }

            if (ignoreFix || myGNSS.getPVT() && (myGNSS.getFixType() != 0) && (myGNSS.getSIV() > 3)) {
                gpsFix = true;

                checkRegionByLocation();

                if(lastLoRaWANRegion != Lorawan_Geofence_region_code) {
                    SerialUSB.println(F("Region has changed, force LoRaWAN ABP Init")); 
                    ABPInitStatus = false;
                    lastLoRaWANRegion = Lorawan_Geofence_region_code;
                }
          
                if(!ABPInitStatus && (Lorawan_Geofence_no_tx == 0)){            
                    SerialUSB.println(F("LoRaWAN ABP initiated..."));     
                    initThroughABP();
                    SerialUSB.println(F("LoRaWAN ABP Initialize success..."));     
                    ABPInitStatus = true;
                }


                updateTelemetry();
                #if defined(DEVMODE)        
                SerialUSB.print(F("Telemetry Size: "));
                SerialUSB.print(telemetry.getSize());
                SerialUSB.println(F(" bytes"));

                Serial.print(F("Telemetry Size: "));
                Serial.print(telemetry.getSize());
                Serial.println(F(" bytes"));
                #endif

                //need to save power
                if (readBatt() < gpsMinVolt) {
                    GpsOFF;
                    ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
                    gpsFix = false;
                    delay(500);      
                }

                if (Lorawan_Geofence_no_tx == 0) {
                    sendLoRaWANPacket();
                    SerialUSB.println(F("LoRaWAN packet sent.."));  

                    // flash three times to send out signal
                    for (int i = 1; i <= 3; i++) {
                        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
                        delay(300);                     // wait for a second
                        digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
                        delay(300);                     // wait for a second
                    }
                }   
                // // add delay before launch by Sean Lin
                // delay(55000);
            } else {
                SerialUSB.println(F("No GPS fix yet..."));
                fetchGNSSData();
                fetchBMP581Data();
                fetchBatteryVoltage();

                // flash twice to show no gps is fixed yet
                for (int i = 1; i <= 2; i++) {
                    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
                    delay(500);                     // wait for 0.5 second
                    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
                    delay(500);                     // wait for 0.5 second
                }
            }
        }
        //this code block protecting serial connected (3V + 3V) super caps from overcharging by powering on GPS module.
        //GPS module uses too much power while on, so if voltage is too high for supercaps, GPS ON.
        if (readBatt() > 6.5) {
            GpsON;
            delay(500);
        }
    } else {
        GpsOFF;
        ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.     
        gpsFix=false;

        #if defined(DEVMODE)
        SerialUSB.print("GPS off, supply voltage= ");
        SerialUSB.print(voltage);
        SerialUSB.print(" V\t\t");
        SerialUSB.println();  // adds a newline after the prints
        #endif

        // flash LED once to show that it is in sleep mode
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
        delay(500);                      // wait for 0.5 second
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
        delay(500);                      // wait for 0.5 second

        delay(battWait * 1000);
    }
    delay(1000);
}

void setup_GPS_BMP() {
    GpsON;
    delay(500);
    
    Wire.begin();

    if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
    {
        SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
        ;
    }

    // do not overload the buffer system from the GPS, disable UART output
    myGNSS.setUART1Output(0); //Disable the UART1 port output 
    myGNSS.setUART2Output(0); //Disable Set the UART2 port output
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

    //myGNSS.enableDebugging(); //Enable debug messages over Serial (default)

    myGNSS.setNavigationFrequency(2);//Set output to 2 times a second. Max is 10
    byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
    SerialUSB.print("Current update rate for GPS: ");
    SerialUSB.println(rate);

    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU); // Disable BeiDou
    myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS); // Enable GLONASS

    myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

    initBMP581();
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
        ublox_high_alt_mode_enabled = true;
        #if defined(DEVMODE)
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGNSS.getDynamicModel());
        #endif
    }
}

void initBMP581()
{
    // Initialize sensor connection with retry mechanism
    int8_t err = BMP5_OK;
    unsigned long retryInterval = 1000; // Retry interval in milliseconds
    int retryCount = 10;                // Number of retries before giving up

    // Try initializing the sensor
    while (retryCount > 0)
    {
        err = pressureSensor.beginI2C(i2cAddress);

        if (err == BMP5_OK)
        {
            break; // Successfully connected, exit loop
        }

        // Not connected, inform user and retry
        SerialUSB.println("Error: BMP581 not connected, check wiring and I2C address!");
        SerialUSB.print("Error code: ");
        SerialUSB.println(err);

        retryCount--;
        if (retryCount > 0)
        {
            SerialUSB.print("Retrying in ");
            SerialUSB.print(retryInterval / 1000);
            SerialUSB.println(" seconds...");
            delay(retryInterval);
        }
    }

    if (err != BMP5_OK)
    {
        SerialUSB.println("Failed to initialize BMP581 sensor after multiple attempts.");
    }

    SerialUSB.println("BMP581 connected!");
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

void fetchBatteryVoltage()
{
    voltage = readBatt();
    SerialUSB.print("Battery voltage: ");
    SerialUSB.print(voltage);
    SerialUSB.println(" V");
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
    
    if (ENABLE_RANDOM_LETTER_SENDING) {
        // random letter sequence
        if (!sentenceSent) {
            for (int i = 0; i < sentence_length; i++) {
                indices[i] = i;
            }
        shuffleIndices(indices, sentence_length);
        sentenceSent = true;
        letter_index_in_random_sequence = 0;
        SerialUSB.println("New random sequence generated");
        }

        // send the next letter in the random sequence
        if (letter_index_in_random_sequence < sentence_length) {
            int char_index_random = indices[letter_index_in_random_sequence];
            char letter_to_send  = my_sentence.charAt(char_index_random);

            telemetry.addDigitalInput(9, char_index_random);
            telemetry.addDigitalInput(10, letter_to_send);

            SerialUSB.print("Char index in sentence: ");
            SerialUSB.println(char_index_random);
            SerialUSB.print("Sending letter: ");
            SerialUSB.println(letter_to_send);
            SerialUSB.print("Index in random sequence: ");
            SerialUSB.println(letter_index_in_random_sequence);

            letter_index_in_random_sequence++;
        } else {
            telemetry.addDigitalInput(9, 0xFF);
            telemetry.addDigitalInput(10, 0xFF);

            sentenceSent = false;
            SerialUSB.println("All letters sent");
        }
    }
}

// Fisher-Yates shuffle algorithm
void shuffleIndices(int arr[], int n) {
    for (int i = n - 1; i > 0; i--) {
      int j = random(i + 1); // generate a random number [0, i]
      int temp = arr[i];
      arr[i] = arr[j];
      arr[j] = temp;
    }
  }

void checkRegionByLocation() {
#ifdef DEVMODE
    // us915_0
    // float tempLat = 37.315000;
    // float tempLong = -122.031200;

    // eu868
    float tempLat = 0;
    float tempLong = 51.5;
#else
    float tempLat = myGNSS.getLatitude() / 10000000.f;
    float tempLong = myGNSS.getLongitude() / 10000000.f;
#endif
    
    // Check if the current location is inside the geofence
    // modify: Lorawan_Geofence_no_tx 
    //         Lorawan_Geofence_region_code
	//	       Lorawan_Geofence_special_region_code
    Lorawan_Geofence_position(tempLat,tempLong);
    
  }

void os_getJoinEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getNwkKey(u1_t *buf) { memcpy_P(buf, NWKSKEY, 16); }

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

void initThroughABP() {
    SerialUSB.println(F("Initializing E22-900M22S (LMIC) through ABP..."));

    os_init(nullptr); // LMIC initialization
    LMIC_reset();

    // Set the session keys
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    //DO NOT CHANGE following code blocks unless you know what you are doing :)

    //Europe
    if(Lorawan_Geofence_region_code == _REGCODE_EU868) {

        SerialUSB.println(F("Region EU868"));
  
        //A little hack for Russian region since BasicMAC does not officially support RU864-870. Tested on TTN and worked..   
        if(Lorawan_Geofence_special_region_code == _REGCODE_RU864) {
          SerialUSB.println(F("Special Region RU864"));
          LMIC_setupChannel(0, 868900000, DR_RANGE_MAP(0, 2));
          LMIC_setupChannel(1, 869100000, DR_RANGE_MAP(0, 2));      
        } 
         //DR2 (SF10 BW125kHz)
         //SF10 is better/optimum spreading factor for high altitude balloons
         LMIC_setDrTxpow(2,KEEP_TXPOWADJ);        
  
       //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
        } else if (Lorawan_Geofence_region_code == _REGCODE_AS923) {
          SerialUSB.println(F("Region AS923"));
          
       //A little hack for Korean region since BasicMAC does not officially support KR920-923. Tested on TTN and worked..
        if(Lorawan_Geofence_special_region_code == _REGCODE_KR920) {
          SerialUSB.println(F("Special Region KR920"));        
          LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(0, 2));
          LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(0, 2));
          LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(0, 2));
        }
             
         //DR2 (SF10 BW125kHz)
         //For AS923, DR2 join only since max payload limit is 11 bytes.
         LMIC_setDrTxpow(2,KEEP_TXPOWADJ); 
  
        //North and South America (Except Brazil)
        } else if (Lorawan_Geofence_region_code == _REGCODE_US915) {
  
          SerialUSB.println(F("Region US915"));
  
         //DR0 (SF10 BW125kHz)
         //For US915, DR0 join only since max payload limit is 11 bytes.
         LMIC_setDrTxpow(0,KEEP_TXPOWADJ);
         //TTN and Helium only supports second sub band (channels 8 to 15)
         //so we should force BasicMAC to initiate a join with second sub band channels.
         LMIC_selectChannel(8); 
        
        //Australia and New Zeleand   
        } else if (Lorawan_Geofence_region_code == _REGCODE_AU915) {
  
           SerialUSB.println(F("Region AU915"));
         //DR2 (SF10 BW125kHz)
         //For AU915, DR2 join only since max payload limit is 11 bytes.
         LMIC_setDrTxpow(2,KEEP_TXPOWADJ);
         //TTN and Helium only supports second sub band (channels 8 to 15)
         //so we should force BasicMAC to initiate a join with second sub band channels.
         LMIC_selectChannel(8);
          
        } else {
          
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);  
          
        }
    // Disable ADR
    LMIC_setAdrMode(false);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    SerialUSB.println(F("LoRaWAN ABP initialization successful!"));
}

// Telemetry size is very important, try to keep it lower than 51 bytes. Always lower is better.
void sendLoRaWANPacket()
{
    checkRegionByLocation();
    SerialUSB.print(F("Region: "));
    SerialUSB.println(Lorawan_Geofence_region_code);

    //Europa
    if(Lorawan_Geofence_region_code == _REGCODE_EU868) {

        if(telemetry.getSize() < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (telemetry.getSize() < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);   
            
            }
      //Japan, Malaysia, Singapore, Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
      }  else if (Lorawan_Geofence_region_code == _REGCODE_AS923) {

        if(telemetry.getSize() < 54) {
             //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
          } else {
            //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(5,KEEP_TXPOWADJ);   
            
            }   

      //North and South America (Except Brazil) or Australia and New Zeleand 
      } else if (Lorawan_Geofence_region_code == _REGCODE_US915 || Lorawan_Geofence_region_code == _REGCODE_AU915) {

        //North and South America (Except Brazil)
        if (Lorawan_Geofence_region_code == _REGCODE_US915){
          
          if(telemetry.getSize() < 54) {
             //DR1 (SF9 BW125kHz) max payload size is 53 bytes.
              LMIC_setDrTxpow(1,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR2 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);                                        
          } else {
            //DR3 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);               
          }          

        //Australia and New Zeleand                       
        } else if (Lorawan_Geofence_region_code == _REGCODE_AU915){
          
          if(telemetry.getSize() < 54) {
             //DR3 (SF9 BW125kHz) max payload size is 53 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);         
          } else if (telemetry.getSize() < 126){
            //DR4 (SF8 BW125kHz) max payload size is 125 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);                                        
          } else {
            //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(5,KEEP_TXPOWADJ);             
            }          
        }
        
       
       //TTN and Helium only supports second sub band (channels 8 to 15)
       //so we should force BasicMAC use second sub band channels.
       LMIC_selectChannel(channelNoFor2ndSubBand);
       
       ++channelNoFor2ndSubBand;
       if(channelNoFor2ndSubBand > 15) {
          channelNoFor2ndSubBand = 8;
        }
     //India     
     } else if (Lorawan_Geofence_region_code == _REGCODE_IN865) {

        if(telemetry.getSize() < 52) {
            //DR2 (SF10 BW125kHz) max payload size is 51 bytes.
            LMIC_setDrTxpow(2,KEEP_TXPOWADJ);          
          } else if (telemetry.getSize() < 116){
            //DR3 (SF9 BW125kHz) max payload size is 115 bytes.
            LMIC_setDrTxpow(3,KEEP_TXPOWADJ);                                        
          } else {
            //DR4 (SF8 BW125kHz) max payload size is 222 bytes.
            LMIC_setDrTxpow(4,KEEP_TXPOWADJ);   
            
            }
      }
    
    LMIC_selectChannel(channelNoFor2ndSubBand);

    ++channelNoFor2ndSubBand;
    if (channelNoFor2ndSubBand > 7)
    {
        channelNoFor2ndSubBand = 0;
    }

    if (Lorawan_Geofence_region_code == _REGCODE_EU868) {
        //DR5 (SF7 BW125kHz) max payload size is 222 bytes.
        LMIC_setDrTxpow(5,KEEP_TXPOWADJ);

        //Channel 0, 868.1 MHz
        channelNoFor2ndSubBand = 0;
        LMIC_selectChannel(channelNoFor2ndSubBand);
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
