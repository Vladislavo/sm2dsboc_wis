#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <bus_protocol/bus_protocol.h>

#include <SoftwareSerial.h>

#include "util/log/log.h"

#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define BAUDRATE                            115200
#define DHT22_PIN                           A0
#define DHT22_READ_RETRIES                  20

#define DATA_SEND_PERIOD                    60000

//extern HardwareSerial Serial;
SoftwareSerial debug(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);

DHT dht(DHT22_PIN, DHT22);

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_WIS;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    uint16_t soil_moisture_2 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
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

void setup() {
    Serial.begin(BAUDRATE);
    debug.begin(BAUDRATE);

    dht.begin();
    randomSeed(analogRead(A1));
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
    sensors_data->soil_moisture_2 = 0;

    do {
        sensors_data->dht_temp = dht.readTemperature();
        sensors_data->dht_hum = dht.readHumidity();
        if (isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum)) {
            LOG_E(F("Failed to read from DHT sensor!\n"));
            dht22_read_ctr++;
            delay(100);
        }
    } while ((isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum)) &&
             dht22_read_ctr < DHT22_READ_RETRIES);

    LOG_D(F("A0: ")); LOG_D(sensors_data->soil_moisture_0);
    LOG_D(F(" A1: ")); LOG_D(sensors_data->soil_moisture_1);
    LOG_D(F(" A2: ")); LOG_D(sensors_data->soil_moisture_2);
    LOG_D(F("\n"));
    LOG_D(F("Humidity: "));
    LOG_D(sensors_data->dht_hum);
    LOG_D(F("%  Temperature: "));
    LOG_D(sensors_data->dht_temp);
    LOG_D(F("°C "));
    LOG_D(F("\n"));
}

uint8_t send_data(const sensors_data_t *sensors_data, const int8_t retries) {
    uint8_t ret = 0;
    uint8_t retr = 0;

    uint8_t packet_buffer[32] = {0};
    uint8_t packet_buffer_length = 0;
    uint8_t payload[32] = {0};
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

    memcpy(&packet[*packet_length], &sensors_data->soil_moisture_2, sizeof(sensors_data->soil_moisture_2));
    (*packet_length) += sizeof(sensors_data->soil_moisture_2);

    memcpy(&packet[*packet_length], &sensors_data->dht_temp, sizeof(sensors_data->dht_temp));
    (*packet_length) += sizeof(sensors_data->dht_temp);

    memcpy(&packet[*packet_length], &sensors_data->dht_hum, sizeof(sensors_data->dht_hum));
    (*packet_length) += sizeof(sensors_data->dht_hum);
}
