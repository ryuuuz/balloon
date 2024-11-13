#include "esp_camera.h"
#include <WiFi.h>

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/*reset=*/U8X8_PIN_NONE);

static char recv_buf[512];
static bool is_exist = false;
static bool is_join = false;

static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
    int ch = 0;
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

    if (p_ack == NULL)
    {
        return 0;
    }

    do
    {
        while (Serial1.available() > 0)
        {
            ch = Serial1.read();
            recv_buf[index++] = ch;
            Serial.print((char)ch);
            delay(2);
        }

        if (strstr(recv_buf, p_ack) != NULL)
        {
            return 1;
        }

    } while (millis() - startMillis < timeout_ms);
    return 0;
}

static int image_send()
{
    int ret = 0;
    char cmd[256];

    // Capture image frame buffer
    esp_camera_fb_return(esp_camera_fb_get()); // Return old frame buffer to driver
    camera_fb_t *fb = esp_camera_fb_get();     // Get new frame buffer
    if (!fb)
    {
        Serial.println("Camera capture failed");
        delay(1000);
        return -1;
    }

    uint8_t *jpeg_buf = NULL;
    size_t jpeg_buf_size = 0;
    ret = fmt2jpg(fb->buf, fb->len, 96, 96, PIXFORMAT_RGB565, 32, &jpeg_buf, &jpeg_buf_size);
    if (ret == 0 || jpeg_buf_size == 0)
    {
        Serial.println("JPEG encoding failed");
        esp_camera_fb_return(fb);
        return -1;
    }

    Serial.printf("Image size: %zu\n", jpeg_buf_size);

    // Total image size
    size_t total_size = jpeg_buf_size;

    // Get the maximum packet length available to send
    int available_length = 0;

    // Continue sending while there is still data to send
    size_t data_offset = 0; // Data offset to send from jpeg_buf
    uint16_t packet_idx = 0;

    while (data_offset < total_size)
    {
        // Get available length with AT+LW=LEN
        ret = at_send_check_response("OK", 1000, "AT+LW=LEN\r\n");
        if (ret != 0)
        {
            Serial.println("Failed to get available length");
            return -1;
        }

        // Print and extract available length from response
        Serial.printf("Response: %s\n", recv_buf); // Directly use recv_buf

        char *comma_pos = strchr(recv_buf, ','); // Find the comma in response
        if (comma_pos != NULL)
        {
            // Parse the number after the comma (length)
            available_length = atoi(comma_pos + 1); // +1 to skip the comma
            Serial.printf("Available length: %d\n", available_length);

            // Ensure available_length is valid and within a reasonable range
            if (available_length <= 0 || available_length > 100)
            {
                Serial.println("Invalid available length");
                return -1;
            }
        }
        else
        {
            Serial.println("Failed to parse available length");
            return -1;
        }

        // Subtract 2 bytes for the header (packet[0] and packet[1])
        size_t data_to_send = available_length - 2; 

        // Calculate the bytes to copy based on available length and remaining data
        size_t bytes_to_copy = std::min(static_cast<size_t>(data_to_send), total_size - data_offset);

        // Prepare the packet (header + data)
        uint8_t packet[1024] = {0};  // Max size 1024 (1 byte header + 100 bytes data)
        packet[0] = packet_idx;      // Current packet number

        // Determine if it's the last packet
        if (data_offset + bytes_to_copy >= total_size)
        {
            packet[1] = 1; // Last packet
        }
        else
        {
            packet[1] = 0; // Not the last packet
        }

        // Copy JPEG data to the packet, considering current data offset
        memcpy(&packet[2], jpeg_buf + data_offset, bytes_to_copy);

        // Build command string for AT+MSGHEX
        sprintf(cmd, "AT+MSGHEX=\"");
        for (size_t i = 0; i < bytes_to_copy + 2; i++) // +2 for the header
        {
            sprintf(cmd + strlen(cmd), "%02X", packet[i]);
        }
        strcat(cmd, "\"\r\n");

        // Send the current packet
        Serial.printf("Sending packet %d/%d, size: %d\n", packet_idx + 1, (total_size + available_length - 1) / available_length, bytes_to_copy);
        ret = at_send_check_response("Done", 5000, cmd);
        if (ret != 0)
        {
            Serial.printf("Failed to send packet %d\n", packet_idx);
            break;
        }

        // Move the data offset to the next block of data
        data_offset += bytes_to_copy;
        packet_idx++;
    }

    // Release JPEG buffer and frame buffer
    free(jpeg_buf);
    esp_camera_fb_return(fb);

    return ret;
}

void setup()
{
    u8x8.begin();
    u8x8.setFlipMode(1);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("LoRa Image Transfer Test");

    Serial.begin(115200);
    // Serial.setDebugOutput(true);

    Serial1.begin(9600, SERIAL_8N1, 44, 43);

    if (at_send_check_response("+AT: OK", 100, "AT\r\n"))
    {
        is_exist = true;
        at_send_check_response("+ID: DevEui", 1000, "AT+ID=DevEui,\"3fbe235ebdbfc6a8\"\r\n");                   // replace 'xxxxxxxxxxxxx' with your DevEui
        at_send_check_response("+ID: AppEui", 1000, "AT+ID=AppEui,\"8000000000000007\"\r\n");                   // replace 'xxxxxxxxxxxxx' with your AppEui
        at_send_check_response("+KEY: APPKEY", 1000, "AT+KEY=APPKEY,\"9ed1210d0f14afb6c40528e2d2857375\"\r\n"); // replace 'xxxxxxxxxxxxx' with your AppKey
        at_send_check_response("+ID: DevAddr", 1000, "AT+ID=DevAddr\r\n");
        at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
        at_send_check_response("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");
        at_send_check_response("+DR: US915", 1000, "AT+DR=US915\r\n"); // Change FREQ as per your location
        at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-7\r\n");
        at_send_check_response("+CLASS: C", 1000, "AT+CLASS=A\r\n");
        at_send_check_response("+PORT: 8", 1000, "AT+PORT=8\r\n");
        delay(200);
        u8x8.setCursor(5, 0);
        u8x8.print("LoRaWAN");
        is_join = true;
    }
    else
    {
        is_exist = false;
        Serial.print("No E5 module found.\r\n");
        u8x8.setCursor(0, 1);
        u8x8.print("unfound E5 !");
    }

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
    config.xclk_freq_hz = 10000000;
    config.frame_size = FRAMESIZE_96X96;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 1;

    // 初始化相机
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

void loop()
{
    if (is_exist)
    {
        int ret = 0;
        if (is_join)
        {
            ret = at_send_check_response("+JOIN: Network joined", 12000, "AT+JOIN\r\n");
            if (ret)
            {
                is_join = false;
            }
            else
            {
                at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
                Serial.print("JOIN failed!\r\n\r\n");
                delay(5000);
            }
        }
        else
        {
            image_send();
            delay(1000 * 60); // 1分钟发送一次
        }
    }
    else
    {
        delay(1000);
    }
    delay(1000); // 1秒后获取下一帧
}