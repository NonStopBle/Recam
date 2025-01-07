/*
# Recam Project
# This project streams camera data from an ESP32-CAM to a Python application over a UDP network in real-time.
# It can be integrated with image detection or other processing tasks for further applications.
# Author: Rezier [https://www.youtube.com/@REZIER_0]
# Github : NonStopBle [https://github.com/NonStopBle/Recam]
# Date: (07/01/2025)
# License: MIT (see accompanying LICENSE file)
*/
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_camera.h>
#include <Arduino.h>

// Replace these with your network credentials
const char *ssid = "your-network-ap";
const char *password = "your-network-password";

// Replace with your PC's IP address
const char *udpAddress = "192.168.10.2"; // Example IP
const int udpPort = 7445;
const int MAX_UDP_PACKET_SIZE = 1400; // Adjust based on network MTU

WiFiUDP udp;
uint32_t frameId = 0; // Initialize frame ID

union IntConvert
{
    int16_t asInt = 0;
    uint8_t asByte[2];
} robotDataSend[5], RobotDataRecv[5];

void setup()
{
    Serial.begin(9600);
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    // Start UDP
    udp.begin(udpPort);
    Serial.printf("UDP started on port %d\n", udpPort);

    // Camera configuration
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 22750000;       // 22750000 -> FHD 20Mhz
    config.pixel_format = PIXFORMAT_JPEG; // PIXFORMAT_JPEG; // Use JPEG to reduce data size

    // Frame parameters
    config.frame_size = FRAMESIZE_VGA; 
    config.jpeg_quality = 15;          // Adjust quality (0-63 lower is better)
    config.fb_count = 2;               // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    // Start UDP
    udp.begin(udpPort);
    Serial.printf("UDP started on port %d\n", udpPort);

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    sensor_t *s = esp_camera_sensor_get();

    // s->set_hmirror(s, 1);
    s->set_brightness(s, 50);
    s->set_exposure_ctrl(s, 1);

    robotDataSend[0].asInt = 0;
    robotDataSend[1].asInt = 0;
    robotDataSend[2].asInt = 0;
    robotDataSend[3].asInt = 0;
    robotDataSend[4].asInt = 0;

    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        while (true)
        {
            delay(1);
        }
    }
    Serial.println("Camera initialized");
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

uint8_t datapackageRecv[10];
static uint64_t previousTime = 0;

void UDPRun()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        delay(10);
        return;
    }

    // Calculate the total number of packets needed for this frame
    int totalPackets = (fb->len + MAX_UDP_PACKET_SIZE - 1) / MAX_UDP_PACKET_SIZE;

    for (int i = 0; i < totalPackets; i++)
    {
        // Calculate remaining bytes
        size_t remaining = fb->len - i * MAX_UDP_PACKET_SIZE;
        // Determine the size of this packet
        size_t packetSize = remaining < MAX_UDP_PACKET_SIZE ? remaining : MAX_UDP_PACKET_SIZE;

        // Create a buffer for the packet
        // Header: 4 bytes frameId, 2 bytes packetSeqNum, 2 bytes totalPackets
        uint8_t buffer[18 + MAX_UDP_PACKET_SIZE];
        // Frame ID (4 bytes, big endian)
        buffer[0] = (frameId >> 24) & 0xFF;
        buffer[1] = (frameId >> 16) & 0xFF;
        buffer[2] = (frameId >> 8) & 0xFF;
        buffer[3] = frameId & 0xFF;
        // Packet Sequence Number (2 bytes, big endian)
        buffer[4] = (i >> 8) & 0xFF;
        buffer[5] = i & 0xFF;
        // Total Packets (2 bytes, big endian)
        buffer[6] = (totalPackets >> 8) & 0xFF;
        buffer[7] = totalPackets & 0xFF;
        // Robot data transmittion
        buffer[8] = robotDataSend[0].asByte[1];
        buffer[9] = robotDataSend[0].asByte[0];

        buffer[10] = robotDataSend[1].asByte[1];
        buffer[11] = robotDataSend[1].asByte[0];

        buffer[12] = robotDataSend[2].asByte[1];
        buffer[13] = robotDataSend[2].asByte[0];

        buffer[14] = robotDataSend[3].asByte[1];
        buffer[15] = robotDataSend[3].asByte[0];

        buffer[16] = robotDataSend[4].asByte[1];
        buffer[17] = robotDataSend[4].asByte[0];

        // Copy image data
        memcpy(buffer + 18, fb->buf + i * MAX_UDP_PACKET_SIZE, packetSize);

        // Send UDP packet
        udp.beginPacket(udpAddress, udpPort);
        udp.write(buffer, 18 + packetSize);
        udp.endPacket();
    }

    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);

    // Increment frame ID for the next frame
    frameId++;

    int packetSize = udp.parsePacket();
    if (packetSize > 0)
    {
        udp.read(datapackageRecv, packetSize);
        // Robot data receiver
        RobotDataRecv[0].asByte[1] = datapackageRecv[0];
        RobotDataRecv[0].asByte[0] = datapackageRecv[1];

        RobotDataRecv[1].asByte[1] = datapackageRecv[2];
        RobotDataRecv[1].asByte[0] = datapackageRecv[3];

        RobotDataRecv[2].asByte[1] = datapackageRecv[4];
        RobotDataRecv[2].asByte[0] = datapackageRecv[5];

        RobotDataRecv[3].asByte[1] = datapackageRecv[6];
        RobotDataRecv[3].asByte[0] = datapackageRecv[7];

        RobotDataRecv[4].asByte[1] = datapackageRecv[8];
        RobotDataRecv[4].asByte[0] = datapackageRecv[9];
    }
}

String serialMessage = "";
void loop()
{
    if(Serial.available()  > 0){
        serialMessage = Serial.readStringUntil('\n');
        robotDataSend[0].asInt = getValue(serialMessage , ',' , 0).toInt();
        robotDataSend[1].asInt = getValue(serialMessage , ',' , 1).toInt();
        robotDataSend[2].asInt = getValue(serialMessage , ',' , 2).toInt();
        robotDataSend[3].asInt = getValue(serialMessage , ',' , 3).toInt();
        robotDataSend[4].asInt = getValue(serialMessage , ',' , 4).toInt();
        serialMessage = "";
    }
    UDPRun();
    Serial.printf("%d %d %d %d %d \n", RobotDataRecv[0].asInt, RobotDataRecv[1].asInt, RobotDataRecv[2].asInt, RobotDataRecv[3].asInt, RobotDataRecv[4].asInt);
}
