#include "JSON_UARTReader.h"

JSON_UARTReader::JSON_UARTReader(HardwareSerial &serial, uint32_t baud_rate,
                                 uint8_t rx_pin, uint8_t tx_pin)
    : uart(serial) {}

DeserializationError JSON_UARTReader::read(JsonDocument &doc) {
  size_t bytes_read = 0;
  while (uart.available() > 0 && bytes_read < UART_BUFFER_SIZE - 1) {
    buffer[bytes_read] = uart.read();
    ++bytes_read;
  }

  buffer[bytes_read] = '\0';
  DeserializationError err = deserializeJson(doc, buffer);

  return err;
}