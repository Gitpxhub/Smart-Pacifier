#include <WiFi.h>
#include <esp_now.h>

typedef struct
{
    int rawFlow;
    float airVelocity;
    int micPeak;
    int micAverage;
} SensorData;

SensorData sensorData;

void OnDataRecv(const esp_now_recv_info_t *recvInfo,
                const uint8_t *incomingData,
                int len)
{
    memcpy(&sensorData, incomingData, sizeof(sensorData));

    Serial.print("Raw Flow : ");
    Serial.print(sensorData.rawFlow);

    Serial.print("  Air Velocity : ");
    Serial.print(sensorData.airVelocity);
    Serial.print(" m/s");

    Serial.print("  Mic Peak : ");
    Serial.print(sensorData.micPeak);

    Serial.print("  Mic Average : ");
    Serial.println(sensorData.micAverage);
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    Serial.print("Master MAC : ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Master Ready");
}

void loop()
{
}