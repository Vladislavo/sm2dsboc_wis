#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <bus_protocol/bus_protocol.h>

#include <SoftwareSerial.h>

#define BUS_PROTOCOL_TRANSMIT_RETRIES       5
#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define DEBUG_SERIAL_RX_PIN                 2
#define DEBUG_SERIAL_TX_PIN                 3

#define BAUDRATE                            115200
#define DHT22_PIN                           A0

#define DATA_SEND_PERIOD                    30000

//extern HardwareSerial Serial;
SoftwareSerial debug(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);

DHT dht(DHT22_PIN, DHT22);

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_WIS;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;

void read_sensors_data(sensors_data_t *sensors_data);
uint8_t send_data(const sensors_data_t *sensors_data);

void bus_protocol_sensor_data_encode(
    const sensors_data_t *sensor_data,
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


}

void loop() {
    read_sensors_data(&sensors_data);

    while (!send_data(&sensors_data)) {
        r_del = rand() % 1000;
        delay(r_del); 
    }

    sleep(DATA_SEND_PERIOD + (rand() % 2000));
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = analogRead(A1);
    sensors_data->soil_moisture_1 = analogRead(A3);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    sensors_data->dht_temp = t;
    sensors_data->dht_hum = h;

    if (isnan(h) || isnan(t)) {
      debug.println(F("Failed to read from DHT sensor!"));
      return;
    }

    debug.print(F("Humidity: "));
    debug.print(h);
    debug.print(F("%  Temperature: "));
    debug.print(t);
    debug.print(F("°C "));
}

uint8_t send_data(const sensors_data_t *sensors_data) {
    uint8_t ret = 0;
    uint8_t retries = 0;

    uint8_t packet_buffer[24] = {0};
    uint8_t packet_buffer_length = 0;

    // send
    // TODO: integrate time
    uint32_t utc = 0;
    bus_protocol_data_send_encode(  (uint8_t *) sensors_data,
                                    sizeof(*sensors_data),
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
            debug.println("ESP ACK");
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
        debug.print(F("WIS TRANSMIT REQUEST"));

        // wait for grant
        if (bus_protocol_serial_receive(packet_buffer, &packet_buffer_length)) {
            // check for granted
            if (bus_protocol_packet_decode(packet_buffer, packet_buffer_length, packet_buffer, &packet_buffer_length) == BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_GRANT &&
                bus_protocol_transmit_grant_decode(packet_buffer, packet_buffer_length) == board_id) 
            {
                granted = 1;
            } else {
                debug.println(F("ESP TRANSMIT NOT GRANT"));
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

    debug.print(F("Received message: "));
    for (uint8_t i = 0; i < *data_length; i++) {
        debug.write(data[i]);
    }
    debug.println();

    return *data_length;
}

void sleep(uint32_t ms) {
    // future possible employ powerdown functions
    delay(ms);
}