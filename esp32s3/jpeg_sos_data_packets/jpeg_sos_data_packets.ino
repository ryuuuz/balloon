#include "esp_camera.h"
#include <WiFi.h>


#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      21

#define TOUCH_PAD_NUM     1     // 触摸传感器引脚 (可以调整为实际引脚)
#define TOUCH_THRESHOLD   50000 // 触摸阈值

#define PACKAGE_SIZE 200  // 每个包的大小（字节）
#define START_MARKER 0xAA // 每包开头标记
#define END_MARKER 0x55   // 每包结尾标记

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial1.begin(115200, SERIAL_8N1, 44, 43); // 用于通信的串口

    // 相机配置
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    // config.frame_size = FRAMESIZE_240X240;
    config.frame_size = FRAMESIZE_QQVGA;
    // config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 20;
    config.fb_count = 2;

    // 初始化相机
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    // 初始化触摸传感器
    // touchAttachInterrupt(TOUCH_PAD_NUM, nullptr, TOUCH_THRESHOLD);

    // 打印相机信息
    sensor_t *s = esp_camera_sensor_get();
    Serial.println("==== Camera Info ====");
    Serial.printf("Pixel Format: %s\n", s->pixformat == PIXFORMAT_RGB565 ? "RGB565" : "JPEG");
    Serial.printf("PSRAM found: %s\n", psramFound() ? "true" : "false");
    Serial.println("====================");
}

void loop() {
    // 检查触摸传感器状态
    if (touchRead(TOUCH_PAD_NUM) > TOUCH_THRESHOLD) {
        // 触摸触发拍摄照片
        camera_fb_t *fb = esp_camera_fb_get(); // 捕获照片帧
        if (fb) {
            // 打印帧相关的信息
            Serial.println("==== Frame Info ====");
            Serial.printf("Buffer Length: %u bytes\n", fb->len);
            Serial.printf("Width: %u pixels\n", fb->width);
            Serial.printf("Height: %u pixels\n", fb->height);
            Serial.printf("Format: %s\n", (fb->format == PIXFORMAT_JPEG) ? "JPEG" : "Other");
            Serial.printf("Timestamp: %ld seconds, %ld microseconds since boot\n", (long)fb->timestamp.tv_sec, (long)fb->timestamp.tv_usec);
            Serial.println("====================");
            
            Serial.println("Photo taken!");

            // 统计一共发送多少字节
            size_t total_bytes = 0;

            // 分包发送照片数据
            for (size_t i = 0; i < fb->len; i += PACKAGE_SIZE) {
                size_t packet_size = PACKAGE_SIZE;
                if (i + PACKAGE_SIZE > fb->len) { // 如果最后一包不足200字节
                    packet_size = fb->len - i;
                }

                // 发送包开始标记
                Serial1.write(START_MARKER);

                // 发送图像数据包
                Serial1.write(fb->buf + i, packet_size);

                // 发送包结束标记
                Serial1.write(END_MARKER);

                total_bytes += packet_size;
                total_bytes += 2; // 加上两个标记字节

                delay(10); // 适当延迟，确保接收端能够接收
            }

            Serial.println("Photo sent!");
            Serial.printf("Total bytes sent: %u\n", total_bytes);
            esp_camera_fb_return(fb); // 释放帧缓冲区            
        } else {
            Serial.println("Camera capture failed");
        }

        // 等待一段时间，避免连续触发
        delay(500);
    }
}
