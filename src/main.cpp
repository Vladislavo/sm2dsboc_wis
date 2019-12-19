#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <bus_protocol/bus_protocol.h>

#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32

#define BAUDRATE                            115200
#define DHT22_PIN                           A6

#define DATA_SEND_PERIOD                    60000

extern HardwareSerial Serial;

DHT dht(DHT22_PIN, DHT22);

typedef struct {
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;

void read_sensors_data(sensors_data_t *sensors_data);
uint8_t send_data(const sensors_data_t *sensors_data);

uint8_t request_send_data(const board_id_t board_id);
uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length);

void sleep(const uint32_t ms);

sensors_data_t sensors_data;

void setup() {
    Serial.begin(BAUDRATE);

    //pinMode(DHT22_PIN, INPUT);

    dht.begin();

    Serial.println(F("DHTxx test!"));
}

void loop() {
    read_sensors_data(&sensors_data);

    // get permission to send
    if (request_send_data(BUS_PROTOCOL_BOARD_ID_WIS)) {
        send_data(&sensors_data);
    }

    sleep(DATA_SEND_PERIOD);
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = analogRead(A1);
    sensors_data->soil_moisture_1 = analogRead(A3);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    sensors_data->dht_temp = t;
    sensors_data->dht_hum = h;

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
}

uint8_t send_data(const sensors_data_t *sensors_data) {
    uint8_t ret = 0;
    uint8_t retries = 0;

    uint8_t packet_buffer[24] = {0};
    uint8_t packet_buffer_length = 0;

    // send
    // TODO: integrate time
    uint32_t utc = 0;
    bus_protocol_data_send_encode(BUS_PROTOCOL_BOARD_ID_WIS,
                                  utc,
                                  sensors_data->soil_moisture_0,
                                  sensors_data->soil_moisture_1,
                                  sensors_data->dht_temp,
                                  sensors_data->dht_hum,
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
            ret = 1;
        } else {
            retries++;
        }

    } while(!ret && retries < BUS_PROTOCOL_TRANSMIT_RETRIES);

    return ret;
}

#define BUS_PROTOCOL_TRANSMIT_GRANT_SIZE  4
#define BUS_PROTOCOL_MAX_WAITING_TIME     300
uint8_t request_send_data(const board_id_t board_id) {
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
            if (bus_protocol_packet_decode(packet_buffer, packet_buffer_length, packet_buffer, &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_GRANT &&
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

uint8_t bus_protocol_serial_receive(uint8_t *data, uint8_t *data_length) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + BUS_PROTOCOL_MAX_WAITING_TIME < millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
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