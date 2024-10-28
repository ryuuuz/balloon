#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

#define BUFFER_SIZE 128  // 定义缓冲区大小

// 初始化 OLED 显示
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

// 用于存储从 Serial1 接收到的数据的缓冲区
char receivedData[BUFFER_SIZE];
int dataLength = 0;  // 当前数据长度

void setup(void) {
  // 初始化 OLED
  u8x8.begin();
  u8x8.setFlipMode(1);   // 屏幕旋转 180 度

  // 开机显示信息
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 0);
  u8x8.print("Waiting...");

  // 初始化调试串口
  Serial.begin(115200);
  
  // 初始化 Serial1 串口，波特率为 115200
  Serial1.begin(115200);
}

void loop(void) {
  // 检查 Serial1 上是否有数据
  while (Serial1.available() > 0) {
    // 读取 Serial1 中的一个字符
    char incomingChar = Serial1.read();
    
    // 如果缓冲区未满，继续存储字符
    if (dataLength < BUFFER_SIZE - 1) {
      receivedData[dataLength] = incomingChar;
      dataLength++;
    }

    // 如果接收到换行符或缓冲区已满，则认为一条完整的数据已经接收
    if (incomingChar == '\n' || dataLength >= BUFFER_SIZE - 1) {
      // 添加字符串终止符
      receivedData[dataLength] = '\0';
      
      // 打印接收到的数据到调试串口
      Serial.print("Received from Serial1: ");
      Serial.println(receivedData);

      // 清除 OLED 显示并设置字体
      u8x8.clear();
      u8x8.setFont(u8x8_font_chroma48medium8_r);

      // OLED 显示收到的数据，限制显示的最大长度为16字符（两行最大容纳字符数）
      u8x8.setCursor(0, 0);
      for (int i = 0; i < dataLength && i < 16; i++) {
        u8x8.print(receivedData[i]);
      }

      // 清空缓冲区以接收下一条数据
      dataLength = 0;
    }
  }
}
