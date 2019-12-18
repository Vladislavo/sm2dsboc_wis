#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "bus_protocol/bus_protocol.h"

#define BAUDRATE    115200
#define DHT22_PIN   A6

extern HardwareSerial Serial;

DHT dht(DHT22_PIN, DHT22);

char buffer[50] = "";

void send_data(
    const uint16_t soil_moisture_0, 
    const uint16_t soil_moisture_1, 
    const float dht_temp, 
    const float dht_hum);

uint8_t request_send_data(board_id_t board_id);

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length);

void setup() {
    Serial.begin(BAUDRATE);

    //pinMode(DHT22_PIN, INPUT);

    dht.begin();

    Serial.println(F("DHTxx test!"));
}

void loop() {
    //sprintf(buffer, "ADC1 = %d, ADC3 = %d\r\n", analogRead(A1), analogRead(A3));
    //Serial.print(buffer);
    
    delay(2000);

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true);

    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    float hif = dht.computeHeatIndex(f, h);
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
}

void send_data(
    const uint16_t soil_moisture_0, 
    const uint16_t soil_moisture_1, 
    const float dht_temp, 
    const float dht_hum) 
{
    uint8_t packet_buffer[24] = {0};
    uint8_t packet_buffer_length = 0;

    // get permission to send
    
    // send
    
    // TODO: integrate time
    uint32_t utc = 0;
    bus_protocol_data_send_encode(BUS_PROTOCOL_BOARD_ID_WIS,
                                  utc,
                                  soil_moisture_0,
                                  soil_moisture_1,
                                  dht_temp,
                                  dht_hum,
                                  packet_buffer,
                                  &packet_buffer_length);


}

#define BUS_PROTOCOL_TRANSMIT_GRANT_SIZE  4
#define BUS_PROTOCOL_MAX_WAITING_TIME     300
#define BUS_PROTOCOL_TRANSMIT_RETRIES     5
uint8_t request_send_data(board_id_t board_id) {
    uint8_t granted = 0;
    uint8_t retries = 0;

    uint8_t packet_buffer[12] = {0};
    uint8_t packet_buffer_length = 0;
    
    bus_protocol_transmit_request_encode(board_id, packet_buffer, &packet_buffer_length);

    do {
        Serial.write(packet_buffer, packet_buffer_length);

        // wait for grant
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length)) {
            // check for granted
            if (bus_protocol_packet_decode(packet_buffer, &packet_buffer_length, packet_buffer, &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_GRANT &&
                bus_protocol_transmit_grant_decode(packet_buffer, packet_buffer_length) == board_id) 
            {
                granted = 1;
            }
        } else {
            retries++;
        }

    } while(!granted && retries < BUS_PROTOCOL_TRANSMIT_RETRIES);

    return granted;
}

#define BUS_PROTOCOL_MAX_DATA_SIZE        32
uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + BUS_PROTOCOL_MAX_WAITING_TIME < millis() && *data_length < sizeof(BUS_PROTOCOL_MAX_DATA_SIZE)) {
        if (Serial.available()) {
            data[(*data_length)++] = Serial.read();
            // update wating time
            start_millis = millis();
        }
    }

    return *data_length;
}