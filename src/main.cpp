#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <Wire.h>
#include "SHTSensor.h"
#include <HIHReader.h>

#include <bus_protocol/bus_protocol.h>

#include <SoftwareSerial.h>

#include "util/log/log.h"

#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define LED_SERIAL                          DD6

#define BAUDRATE                            115200
#define DHT22_PIN                           8
#define DHT22_READ_RETRIES                  100

#define DATA_SEND_PERIOD                    600000

//extern HardwareSerial Serial;
SoftwareSerial debug(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_WIS;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    float dht22_temp = 0;
    float dht22_hum = 0;
    float sht85_temp = 0;
    float sht85_hum = 0;
    float hih8121_temp = 0;
    float hih8121_hum = 0;
} sensors_data_t;

void read_sensors_data(sensors_data_t *sensors_data);
uint8_t send_data(const sensors_data_t *sensors_data, const int8_t retries);

void bus_protocol_sensor_data_payload_encode(
    const sensors_data_t *sensors_data,
    uint8_t *packet,
    uint8_t *packet_length);
uint8_t request_send_data(const board_id_t board_id);
uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length);

void sleep(const uint32_t ms);

sensors_data_t sensors_data;
uint16_t r_del = 0;

float hum, temp;

void setup() {
    Serial.begin(BAUDRATE);
    debug.begin(BAUDRATE);
    Wire.begin();

    pinMode(LED_SERIAL, OUTPUT);
    digitalWrite(LED_SERIAL, LOW);

    dht.begin();
    randomSeed(analogRead(A1));

    sht85.init();
    sht85.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x
}

void loop() {
    do {
        delay(random(5000)); 
        read_sensors_data(&sensors_data);
    } while (!send_data(&sensors_data, BUS_PROTOCOL_TRANSMIT_RETRIES));

    sleep(DATA_SEND_PERIOD + random(5000));
}

void read_sensors_data(sensors_data_t *sensors_data) {
    uint8_t dht22_read_ctr = 0;
    sensors_data->board_id = BUS_PROTOCOL_BOARD_ID_WIS;

    sensors_data->soil_moisture_0 = analogRead(A1);
    sensors_data->soil_moisture_1 = analogRead(A3);

    do {
        sensors_data->dht22_temp = dht.readTemperature();
        sensors_data->dht22_hum = dht.readHumidity();
        if (isnan(sensors_data->dht22_temp) || isnan(sensors_data->dht22_hum)) {
            LOG_E(F("Failed to read from DHT sensor!\n"));
            dht22_read_ctr++;
            delay(100);
        }
    } while ((isnan(sensors_data->dht22_temp) || isnan(sensors_data->dht22_hum)) &&
             dht22_read_ctr < DHT22_READ_RETRIES);

    sht85.readSample();
    sensors_data->sht85_temp = sht85.getTemperature();
    sensors_data->sht85_hum = sht85.getHumidity();

    hih8121.read(&sensors_data->hih8121_temp, &sensors_data->hih8121_hum);

    LOG_D(F("A0: ")); LOG_D(sensors_data->soil_moisture_0);
    LOG_D(F(" A1: ")); LOG_D(sensors_data->soil_moisture_1);
    LOG_D(F("\n"));
    LOG_D(F("DHT22 Humidity: "));
    LOG_D(sensors_data->dht22_hum);
    LOG_D(F("%  Temperature: "));
    LOG_D(sensors_data->dht22_temp);
    LOG_D(F("°C "));
    LOG_D(F("\n"));
}

uint8_t send_data(const sensors_data_t *sensors_data, const int8_t retries) {
    digitalWrite(LED_SERIAL, HIGH);
    uint8_t ret = 0;
    uint8_t retr = 0;

    uint8_t packet_buffer[75] = {0};
    uint8_t packet_buffer_length = 0;
    uint8_t payload[70] = {0};
    uint8_t payload_length = 0;

    bus_protocol_sensor_data_payload_encode(sensors_data, payload, &payload_length);

    // send
    bus_protocol_data_send_encode(  payload,
                                    payload_length,
                                    packet_buffer,
                                    &packet_buffer_length);

    do {
        Serial.write(packet_buffer, packet_buffer_length);

        //wait for ACK
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length) &&
            bus_protocol_packet_decode( packet_buffer, 
                                        packet_buffer_length, 
                                        packet_buffer, 
                                        &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_ACK) 
        {
            LOG_D(F("ESP ACK\n"));
            ret = 1;
        } else {
            retr++;
        }

    } while(!ret && retr < retries);

    digitalWrite(LED_SERIAL, LOW);

    return ret;
}

#define BUS_PROTOCOL_TRANSMIT_GRANT_SIZE  4
#define BUS_PROTOCOL_MAX_WAITING_TIME     3000
uint8_t request_send_data(const board_id_t board_id) {
    uint8_t granted = 0;
    uint8_t retries = 0;

    uint8_t packet_buffer[12] = {0};
    uint8_t packet_buffer_length = 0;
    
    bus_protocol_transmit_request_encode(board_id, packet_buffer, &packet_buffer_length);
    
    do {
        Serial.write(packet_buffer, packet_buffer_length);
        LOG_D(F("WIS TRANSMIT REQUEST\n"));

        // wait for grant
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length)) {
            // check for granted
            if (bus_protocol_packet_decode(packet_buffer, packet_buffer_length, packet_buffer, &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_GRANT &&
                bus_protocol_transmit_grant_decode(packet_buffer, packet_buffer_length) == board_id) 
            {
                granted = 1;
            } else {
                LOG_D(F("ESP TRANSMIT NOT GRANT\n"));
            }
        } else {
            retries++;
        }

    } while(!granted && retries < BUS_PROTOCOL_TRANSMIT_RETRIES);

    return granted;
}

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + BUS_PROTOCOL_MAX_WAITING_TIME > millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
        if (Serial.available()) {
            data[(*data_length)++] = Serial.read();
            // update wating time
            start_millis = millis();
        }
    }

    return *data_length;
}

void sleep(uint32_t ms) {
    // future possible employ powerdown functions
    delay(ms);
}

void bus_protocol_sensor_data_payload_encode(
    const sensors_data_t *sensors_data,
    uint8_t *packet,
    uint8_t *packet_length) 
{
    *packet_length = 0;

    packet[*packet_length] = sensors_data->board_id;
    (*packet_length)++;

    memcpy(&packet[*packet_length], &sensors_data->utc, sizeof(sensors_data->utc));
    (*packet_length) += sizeof(sensors_data->utc);

    memcpy(&packet[*packet_length], &sensors_data->soil_moisture_0, sizeof(sensors_data->soil_moisture_0));
    (*packet_length) += sizeof(sensors_data->soil_moisture_0);

    memcpy(&packet[*packet_length], &sensors_data->soil_moisture_1, sizeof(sensors_data->soil_moisture_1));
    (*packet_length) += sizeof(sensors_data->soil_moisture_1);

    memcpy(&packet[*packet_length], &sensors_data->dht22_temp, sizeof(sensors_data->dht22_temp));
    (*packet_length) += sizeof(sensors_data->dht22_temp);

    memcpy(&packet[*packet_length], &sensors_data->dht22_hum, sizeof(sensors_data->dht22_hum));
    (*packet_length) += sizeof(sensors_data->dht22_hum);

    memcpy(&packet[*packet_length], &sensors_data->sht85_temp, sizeof(sensors_data->sht85_temp));
    (*packet_length) += sizeof(sensors_data->sht85_temp);

    memcpy(&packet[*packet_length], &sensors_data->sht85_hum, sizeof(sensors_data->sht85_hum));
    (*packet_length) += sizeof(sensors_data->sht85_hum);

    memcpy(&packet[*packet_length], &sensors_data->hih8121_temp, sizeof(sensors_data->hih8121_temp));
    (*packet_length) += sizeof(sensors_data->hih8121_temp);

    memcpy(&packet[*packet_length], &sensors_data->hih8121_hum, sizeof(sensors_data->hih8121_hum));
    (*packet_length) += sizeof(sensors_data->hih8121_hum);
}
