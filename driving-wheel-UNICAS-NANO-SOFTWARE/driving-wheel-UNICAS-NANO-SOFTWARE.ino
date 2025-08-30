// UNICAS Driving Wheel Arduino Nano Secondary Board
// By Andrea Roccaccino

// This software reads the car's CAN BUS and can do three things:
// 1) Receive/send frames from the CAN BUS devices (ECU, IMU, AMPMETER, other sensors). This allows to negotiate profiles, collect stats and more.
// 2) Dialog with Arduino UNO R4 WiFi via UART (the driving wheel main MCU). Send stats, receive commands.
// 3) Dialog with the onboard Raspberry Pi. Send stats, receive commands. *This is a backup function just in case the onboard MCP2515 fails because it's kinda difficult to get it up and running on RPi

// Required non-core Libraries: autowp MCP2515
#include <SPI.h>
#include <mcp2515.h>

#define CS_PIN 10
#define INT_PIN 2

struct can_frame canMsg;
MCP2515 mcp2515(CS_PIN);

// LED That will light under the EXTRA button
#define EXTRA_ACTIVITY_LED 5

#include <SoftwareSerial.h>

#define SW_RX 3
#define SW_TX 4

#define nano_serial_baudrate 19200
#define serial_baudrate 115200

SoftwareSerial arduino_uno(SW_RX, SW_TX);  // RX, TX

// Apply packing to ensure there is no padding between struct members
#pragma pack(push, 1)  // Start packing (1-byte alignment)
struct uart_message {
  uint8_t enum_value_action;  // 1 byte
  uint32_t content;           // 4 bytes
};
#pragma pack(pop)  // Restore default packing alignment

constexpr size_t UART_MESSAGE_SIZE = sizeof(uart_message);

enum UART_MESSAGES {
  PROFILE = 0,
  HORN,
  PTT,
  PIT,
  REVERSE,

  ACCEL_X,
  ACCEL_Y,
  ACCEL_Z,
  GYRO_X,
  GYRO_Y,
  GYRO_Z,

  AMPERAGE,

  BATTERY_VOLTAGE,
  THROTTLE,
  BRAKE

};

void send_uart_message(Stream& stream, const uart_message& msg) {
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&msg);
  stream.write(ptr, UART_MESSAGE_SIZE);
}

bool receive_uart_message(Stream& stream, uart_message& msg) {
  static uint8_t buffer[UART_MESSAGE_SIZE];
  static size_t received = 0;

  while (stream.available() > 0 && received < UART_MESSAGE_SIZE) {
    buffer[received++] = stream.read();
  }

  if (received == UART_MESSAGE_SIZE) {
    memcpy(&msg, buffer, UART_MESSAGE_SIZE);
    received = 0;
    return true;
  }

  return false;
}

void loop_uart() {
  uart_message msg;

  // Read from SoftwareSerial
  if (receive_uart_message(arduino_uno, msg)) {
    Serial.print("RX SW: Action=");
    Serial.print(msg.enum_value_action);
    Serial.print(" Content=");
    Serial.println((uint32_t)msg.content);
  }

  // Read from USB Serial
  if (receive_uart_message(Serial, msg)) {
    Serial.print("RX USB: Action=");
    Serial.print(msg.enum_value_action);
    Serial.print(" Content=");
    Serial.println((uint32_t)msg.content);

    switch (msg.enum_value_action) {
      default:

        break;
    }
  }
}

void setup() {
  Serial.begin(serial_baudrate);
  arduino_uno.begin(nano_serial_baudrate);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  Serial.println("Lettore avviato");
}

void loop() {
  loop_uart();

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      digitalWrite(EXTRA_ACTIVITY_LED, 1);
    Serial.print(canMsg.can_id, HEX);  // print ID
    Serial.print(" ");
    Serial.print(canMsg.can_dlc, HEX);  // print DLC
    Serial.print(" ");

    for (int i = 0; i < canMsg.can_dlc; i++) {  // print the data
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();

    if (canMsg.can_id == 0x521) {
      Serial.print("Data received from IVT");
      Serial.print("\t");

      // Convert the amperage data (assuming it's at buf[2], buf[3], buf[4], buf[5])
      long currentReading = (canMsg.data[2] << 24) | (canMsg.data[3] << 16) | (canMsg.data[4] << 8) | (canMsg.data[5]);
      Serial.print("Amperage: ");
      Serial.println(currentReading);

      // Check voltage data (assuming it's at buf[6] and buf[7] for example)
      float voltageReading = (canMsg.data[6] << 8) | canMsg.data[7];     // Combine two bytes for voltage
      voltageReading = voltageReading * (5.0 / 1023.0);  // Example scaling, assuming 10-bit ADC and 5V reference
      Serial.print("Voltage at U1: ");
      Serial.println(voltageReading, 3);  // Print voltage with 3 decimal places
    }

    Serial.write((uint8_t*)&canMsg, sizeof(can_frame));

      digitalWrite(EXTRA_ACTIVITY_LED, 0);
  }
}
