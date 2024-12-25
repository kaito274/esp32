#ifndef __JSON_UARTREADER_H__
#define __JSON_UARTREADER_H__

#include <ArduinoJson.h>

#define UART_BUFFER_SIZE 256

class JSON_UARTReader {
public:
    JSON_UARTReader(HardwareSerial &serial, uint32_t baud_rate, uint8_t rx_pin, uint8_t tx_pin);
    ~JSON_UARTReader() = default;

    DeserializationError read(JsonDocument &doc);
private:
    uint8_t buffer[UART_BUFFER_SIZE] = {0};
    HardwareSerial &uart;
};

#endif // __JSON_UARTREADER_H__