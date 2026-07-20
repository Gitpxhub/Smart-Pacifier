#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "DFRobot_PAV3000.h"

DFRobot_PAV3000 fs;

#define MIC_PIN D1
const int SAMPLE_WINDOW = 50;

uint8_t masterAddress[] =
{
    0x68,0x09,0x47,0x86,0x0a,0x90
};

typedef struct
{
    int rawFlow;
    float airVelocity;
    int micPeak;
    int micAverage;
} SensorData;

SensorData sensorData;

void OnDataSent(const wifi_tx_info_t *info,
                esp_now_send_status_t status)
{
    Serial.print("Send : ");

    if(status==ESP_NOW_SEND_SUCCESS)
        Serial.println("Success");
    else
        Serial.println("Failed");
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(D4,D5);

    pinMode(MIC_PIN,INPUT);

    WiFi.mode(WIFI_STA);

    Serial.print("Slave MAC : ");
    Serial.println(WiFi.macAddress());

    if(esp_now_init()!=ESP_OK)
    {
        Serial.println("ESP-NOW Failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo={};

    memcpy(peerInfo.peer_addr,masterAddress,6);
    peerInfo.channel=0;
    peerInfo.encrypt=false;

    if(esp_now_add_peer(&peerInfo)!=ESP_OK)
    {
        Serial.println("Peer Add Failed");
        return;
    }

    while(!fs.setRange(AIRFLOW_RANGE_15_MPS))
    {
        Serial.println("PAV3015 Not Found");
        delay(500);
    }

    Serial.println("PAV3015 Ready");
}

void loop()
{
    sensorData.rawFlow=fs.readRaw();
    sensorData.airVelocity=fs.readMeterPerSec();

    unsigned long start=millis();

    int maxValue=0;
    int minValue=4095;

    while(millis()-start<SAMPLE_WINDOW)
    {
        int sample=analogRead(MIC_PIN);

        if(sample>maxValue) maxValue=sample;
        if(sample<minValue) minValue=sample;
    }

    sensorData.micPeak=maxValue-minValue;
    sensorData.micAverage=(maxValue+minValue)/2;

    esp_now_send(masterAddress,
                 (uint8_t*)&sensorData,
                 sizeof(sensorData));

    delay(100);
}