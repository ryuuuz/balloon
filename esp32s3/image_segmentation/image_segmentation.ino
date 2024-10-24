#include "esp_camera.h"
#include <WiFi.h>

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// 定义分割大小配置
struct ImageSegmentConfig {
    uint16_t width;
    uint16_t height;
    uint16_t segment_width;
    uint16_t segment_height;
    uint8_t jpeg_quality;
};

framesize_t current_frame_size = FRAMESIZE_240X240;

// 不同信号强度下的配置生成函数
ImageSegmentConfig getConfigForSignal(int signal_strength, framesize_t frame_size) {
    uint16_t width, height;

    // 根据不同分辨率设置宽度和高度
    switch(frame_size) {
        case FRAMESIZE_QVGA:  // 320x240
            width = 320;
            height = 240;
            break;
        case FRAMESIZE_HVGA:  // 480x320
            width = 480;
            height = 320;
            break;
        case FRAMESIZE_VGA:   // 640x480
            width = 640;
            height = 480;
            break;
        case FRAMESIZE_HD:    // 1280x720
            width = 1280;
            height = 720;
            break;
        case FRAMESIZE_SVGA:  // 800x600
            width = 800;
            height = 600;
            break;
        case FRAMESIZE_240X240:  // 240X240
            width = 240;
            height = 240;
            break;
        case FRAMESIZE_96X96:  // 96X96
            width = 96;
            height = 96;
            break;
        default:
            width = 96;  // 默认值
            height = 96;
            break;
    }

    // 根据信号强度动态调整分割配置
    switch(signal_strength) {
        case 2:
            return {width, height, width, height, 10};  // 信号好，发送完整图片
        case 1:
            return {width, height, width / 2, height / 2, 15};  // 信号一般，分割成4份
        default:
            return {width, height, width / 3, height / 3, 20};  // 信号差，分割成9份
    }
}

// 自定义min函数
template<typename T>
T minimum(T a, T b) {
    return (a < b) ? a : b;
}

// 图像分割类
class ImageSegmenter {
private:
    uint8_t* source_buf;
    uint16_t source_width;
    uint16_t source_height;
    ImageSegmentConfig config;

    // 内部函数：创建RGB565缓冲区
    uint8_t* createSegmentBuffer(int start_x, int start_y, int actual_width, int actual_height, size_t* buf_size) {
        *buf_size = actual_width * actual_height * 2; // RGB565 每像素2字节
        uint8_t* rgb_buf = (uint8_t*)malloc(*buf_size);
        if (!rgb_buf) return NULL;

        // 复制分割区域数据
        for (int y = 0; y < actual_height; y++) {
            memcpy(rgb_buf + y * actual_width * 2,
                  source_buf + ((start_y + y) * source_width + start_x) * 2,
                  actual_width * 2);
        }
        return rgb_buf;
    }

public:
    ImageSegmenter(uint8_t* buf, uint16_t width, uint16_t height, ImageSegmentConfig cfg) 
        : source_buf(buf), source_width(width), source_height(height), config(cfg) {}

    // 获取分割后的总段数
    int getTotalSegments() {
        return ((source_width + config.segment_width - 1) / config.segment_width) *
               ((source_height + config.segment_height - 1) / config.segment_height);
    }

    // 提取指定位置的图像段并转换为JPEG
    bool getSegment(int segment_index, uint8_t** jpeg_buf, size_t* jpeg_size) {
        int segments_per_row = (source_width + config.segment_width - 1) / config.segment_width;
        int row = segment_index / segments_per_row;
        int col = segment_index % segments_per_row;

        // 计算分割位置
        int start_x = col * config.segment_width;
        int start_y = row * config.segment_height;

        // 使用自定义minimum函数
        uint16_t actual_width = minimum<uint16_t>(config.segment_width, 
                                                source_width - (uint16_t)start_x);
        uint16_t actual_height = minimum<uint16_t>(config.segment_height, 
                                                 source_height - (uint16_t)start_y);

        // 创建RGB565缓冲区并复制数据
        size_t rgb_buf_size;
        uint8_t* rgb_buf = createSegmentBuffer(start_x, start_y, actual_width, actual_height, &rgb_buf_size);
        if (!rgb_buf) {
            return false;
        }

        // 转换为JPEG
        bool success = fmt2jpg(rgb_buf, rgb_buf_size, actual_width, actual_height, 
                             PIXFORMAT_RGB565, config.jpeg_quality, jpeg_buf, jpeg_size);

        // 释放RGB缓冲区
        free(rgb_buf);
        return success;
    }
};

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial1.begin(115200, SERIAL_8N1, 44, 43); // 用于AT指令的串口

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
    config.frame_size = current_frame_size;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;

    // 初始化相机
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

// 模拟信号强度检测函数
int getSignalStrength() {
    // 这里可以替换为实际的LoRa信号强度检测
    return random(0, 3); // 0:差 1:中 2:好
}

void loop() {
    // 获取一帧图像
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        delay(1000);
        return;
    }

    // 获取当前信号强度
    int signal = getSignalStrength();
    Serial.printf("Signal strength: %d\n", signal);

    // 根据信号强度和当前分辨率选择分割配置
    ImageSegmentConfig current_config = getConfigForSignal(signal, current_frame_size);

    // 创建图像分割器
    ImageSegmenter segmenter((uint8_t*)fb->buf, fb->width, fb->height, current_config);
    int total_segments = segmenter.getTotalSegments();
    Serial.printf("Total segments: %d\n", total_segments);

    // 处理每个分段
    for (int i = 0; i < total_segments; i++) {
        uint8_t* jpeg_buf = NULL;
        size_t jpeg_size = 0;

        if (segmenter.getSegment(i, &jpeg_buf, &jpeg_size)) {
            // 打印分段信息
            Serial.printf("Segment %d/%d, Size: %d bytes\n", i + 1, total_segments, jpeg_size);
            Serial1.printf("Segment %d/%d, Size: %d bytes\n", i + 1, total_segments, jpeg_size);

            // 发送帧头
            Serial1.write(0xAA);  // 帧起始标记
            Serial1.write((uint8_t)(i + 1));  // 分段序号
            Serial1.write((uint8_t)total_segments);  // 总分段数
            Serial1.write((uint8_t)(jpeg_size >> 8));  // 数据长度高字节
            Serial1.write((uint8_t)(jpeg_size & 0xFF));  // 数据长度低字节

            // 发送JPEG数据
            Serial1.write(jpeg_buf, jpeg_size);

            // 发送帧尾
            Serial1.write(0x55);  // 帧结束标记
            
            // 释放JPEG缓冲区
            free(jpeg_buf);
            
            delay(100); // 短暂延时，避免接收端缓冲区溢出
        } else {
            Serial.printf("Failed to process segment %d\n", i + 1);
        }
    }

    esp_camera_fb_return(fb);
    delay(1000); // 1秒后获取下一帧
}