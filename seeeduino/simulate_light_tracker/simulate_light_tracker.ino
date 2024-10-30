
/*
* Grove - Wio-E5 Helium Demo
*  Date: 30/08/2023
* Author: Salman Faris (Modified Seeed TTN Sketch)

*/


#include <Arduino.h>
#include <U8x8lib.h>
#include "DHT.h"


#define DHTTYPE DHT20  // DHT 20

DHT dht(DHTTYPE);

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/*reset=*/U8X8_PIN_NONE);
// U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/*clock=*/ SCL, /*data=*/ SDA, /*reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

static char recv_buf[512];
static bool is_exist = false;
static bool is_join = false;
static int led = 0;
unsigned int counter = 0;  // 初始计数器设为0

static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...) {
  int ch;
  int num = 0;
  int index = 0;
  int startMillis = 0;
  va_list args;
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  Serial1.printf(p_cmd, args);
  Serial.printf(p_cmd, args);
  va_end(args);
  delay(200);
  startMillis = millis();

  if (p_ack == NULL) {
    return 0;
  }

  do {
    while (Serial1.available() > 0) {
      ch = Serial1.read();
      recv_buf[index++] = ch;
      Serial.print((char)ch);
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL) {
      return 1;
    }

  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static void recv_prase(char *p_msg) {
  if (p_msg == NULL) {
    return;
  }
  char *p_start = NULL;
  int data = 0;
  int rssi = 0;
  int snr = 0;

  p_start = strstr(p_msg, "RX");
  if (p_start && (1 == sscanf(p_start, "RX: \"%d\"\r\n", &data))) {
    Serial.println(data);
    u8x8.setCursor(2, 4);
    u8x8.print("led :");
    led = !!data;
    u8x8.print(led);
    if (led) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  p_start = strstr(p_msg, "RSSI");
  if (p_start && (1 == sscanf(p_start, "RSSI %d,", &rssi))) {
    u8x8.setCursor(0, 6);
    u8x8.print("                ");
    u8x8.setCursor(2, 6);
    u8x8.print("rssi:");
    u8x8.print(rssi);
  }
  p_start = strstr(p_msg, "SNR");
  if (p_start && (1 == sscanf(p_start, "SNR %d", &snr))) {
    u8x8.setCursor(0, 7);
    u8x8.print("                ");
    u8x8.setCursor(2, 7);
    u8x8.print("snr :");
    u8x8.print(snr);
  }
}

void setup(void) {
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial1.begin(9600);
  Serial.print("E5 LORAWAN TEST\r\n");
  u8x8.setCursor(0, 0);

  if (at_send_check_response("+AT: OK", 100, "AT\r\n")) {
    is_exist = true;
    at_send_check_response("+ID: DevEui", 1000, "AT+ID=DevEui,\"9e3102be9c89dc08\"\r\n"); // replace 'xxxxxxxxxxxxx' with your DevEui
    at_send_check_response("+ID: AppEui", 1000, "AT+ID=AppEui,\"8000000000000007\"\r\n"); // replace 'xxxxxxxxxxxxx' with your AppEui
    at_send_check_response("+KEY: APPKEY", 1000, "AT+KEY=APPKEY,\"43f3fc5dd3f0273cbf2cf4023dca96cc\"\r\n"); // replace 'xxxxxxxxxxxxx' with your AppKey
    at_send_check_response("+ID: DevAddr", 1000, "AT+ID=DevAddr\r\n"); 
    at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
    at_send_check_response("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");
    at_send_check_response("+DR: US915", 1000, "AT+DR=US915\r\n");  // Change FREQ as per your location
    at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-7\r\n");
    at_send_check_response("+CLASS: C", 1000, "AT+CLASS=A\r\n");
    at_send_check_response("+PORT: 8", 1000, "AT+PORT=8\r\n");
    delay(200);
    u8x8.setCursor(5, 0);
    u8x8.print("LoRaWAN");
    is_join = true;
  } else {
    is_exist = false;
    Serial.print("No E5 module found.\r\n");
    u8x8.setCursor(0, 1);
    u8x8.print("unfound E5 !");
  }

  // dht.begin();

  u8x8.setCursor(0, 2);
  u8x8.setCursor(2, 2);
  u8x8.print("temp:");

  u8x8.setCursor(2, 3);
  u8x8.print("humi:");

  u8x8.setCursor(2, 4);
  u8x8.print("led :");
  u8x8.print(led);
}

void loop(void) {
  // float temp = 0;
  // float humi = 0;

  // temp = dht.readTemperature();
  // humi = dht.readHumidity();

  // Serial.print("Humidity: ");
  // Serial.print(humi);
  // Serial.print(" %\t");
  // Serial.print("Temperature: ");
  // Serial.print(temp);
  // Serial.println(" *C");

  // u8x8.setCursor(0, 2);
  // u8x8.print("      ");
  // u8x8.setCursor(2, 2);
  // u8x8.print("temp:");
  // u8x8.print(temp);
  // u8x8.setCursor(2, 3);
  // u8x8.print("humi:");
  // u8x8.print(humi);

  if (is_exist) {
    int ret = 0;
    if (is_join) {

      ret = at_send_check_response("+JOIN: Network joined", 12000, "AT+JOIN\r\n");
      if (ret) {
        is_join = false;
      } else {
        at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
        Serial.print("JOIN failed!\r\n\r\n");
        delay(5000);
      }
    } else {
      char cmd[128];
      // sprintf(cmd, "AT+MSGHEX=\"%04X%04X\"\r\n", (int)(temp * 100), (int)(humi * 100));
      sprintf(cmd, "AT+MSGHEX=\"%08X\"\r\n", counter);
      counter++;
      
      ret = at_send_check_response("Done", 5000, cmd);
      // if (ret) {
      //   recv_prase(recv_buf);
      // } else {
      //   Serial.print("Send failed!\r\n\r\n");
      // }
      delay(5000);
    }
  } else {
    delay(1000);
  }
}
