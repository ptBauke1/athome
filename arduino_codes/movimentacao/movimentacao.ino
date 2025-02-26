#include <mcp2515.h>
#include <SPI.h>
#include "messages.h"
/*
To initialize a conection with the MCP2515 CAN module
provide the pin where SPI CS is connected, baudrate and mode

modes:
mcp2515.setNormalMode();
mcp2515.setLoopbackMode();
mcp2515.setListenOnlyMode();

CAN_1000KBPS, MCP_8MHZ

Frame data format
struct can_frame {
  uint32_t can_id; -> 32 bit CAN_ID + EFF/RTR/ERR flags
  uint8_t can_dlc; -> Data Lenght Format (8 bytes)
  uint8_t data[8];
}
Frente: 
motor 1 -> +
motor 2 -> -
motor 3 -> -
motor 4 -> +
*/

enum dir { FORWARD,
           BACKWARDS,
           LEFT,
           RIGHT };
MCP2515 mcp2515(10);

uint16_t enc_value = 0;
uint16_t enc_offset = 0;
uint16_t enc_pos = 0;

bool flag_direita = false;
bool flag_frente = false;
bool flag_esquerda = false;
bool flag_tras = false;

long time = 0;

void setup() {
  // put your setup code here, to run once:
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.begin(9600);
  delay(100);

  time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); 
    double linear_x = 0; 
    double linear_y = 0;
    double angular_z = 0;
    // Extract linear.x, linear.y, and angular.z from the received data
    linear_x = extractValue(data, "x:");
    linear_y = extractValue(data, "y:");
    angular_z = extractValue(data, "z:");
    move_robot(linear_x, linear_y, angular_z);
  }
}

float extractValue(String data, String key) {
  int startIndex = data.indexOf(key);
  if (startIndex == -1) return 0.0;  // Se a chave não for encontrada, retorna 0
  
  startIndex += key.length();
  int endIndex = data.indexOf(",", startIndex);
  
  if (endIndex == -1) {
    endIndex = data.length();  // Para o último valor
  }
  
  String valueStr = data.substring(startIndex, endIndex);
  return valueStr.toFloat();  // Converte para float
}

void set_rpm(uint32_t can_id, long rpm) {
  rpm_msg.can_id = 0x140 + can_id;
  long acelerador = rpm * 3600;
  rpm_msg.data[4] = acelerador & 0xFF;
  rpm_msg.data[5] = (acelerador >> 8) & 0xFF;
  rpm_msg.data[6] = (acelerador >> 16) & 0xFF;
  rpm_msg.data[7] = (acelerador >> 24) & 0xFF;

  if (mcp2515.sendMessage(&rpm_msg) == MCP2515::ERROR_ALLTXBUSY) {
    Serial.print("Couldn't send message! All TX Busy");
    return;
  } else {
    delay(1);
    read_msg();
  }
}

void read_encoder(uint32_t can_id) {
  enc_msg.can_id = 0x140 + can_id;

  mcp2515.sendMessage(&enc_msg);
  read_msg();
  enc_pos = (received_msg.data[3] << 8) | received_msg.data[2];
  enc_value = (received_msg.data[5] << 8) | received_msg.data[4];
  enc_offset = (received_msg.data[7] << 8) | received_msg.data[6];

  //Serial.print("enc_pos = "); Serial.println(enc_pos);
  //Serial.print("enc_value = "); Serial.println(enc_value);
  //Serial.print("enc_offset = "); Serial.println(enc_offset);
}

void read_msg() {
  delay(1);
  if (mcp2515.readMessage(&received_msg) == MCP2515::ERROR_OK) {
    for (int i = 0; i < received_msg.can_dlc; i++) {
      Serial.print(received_msg.data[i], HEX);
      Serial.print("\t");
    }
    Serial.println(received_msg.can_id, HEX);
  } else {
    Serial.print("Couldn't read the message! ");
    Serial.println(received_msg.data[0], HEX);
  }
}

void clear_buffer() {
  while (mcp2515.readMessage(&received_msg) == MCP2515::ERROR_OK) {
    Serial.println("Clearing buffer...");
  }
  Serial.println("Buffer cleared!");
}

void move_robot(float vx, float vy, float omega) {
  float R = 0.2032;
  float L = 0.2;
  float W = 0.199;
  float conversion_factor = 60.0 / (2.0 * 3.1415 * R);

  float rpm1 = (-1)*(round((vx - vy - omega * (L+W)) * conversion_factor));
  float rpm2 = round((vx + vy + omega * (L+W)) * conversion_factor);
  float rpm3 = (-1)*(round((vx + vy - omega * (L+W)) * conversion_factor));
  float rpm4 = round((vx - vy + omega * (L+W)) * conversion_factor);

  if (rpm1 == 0 && rpm2 == 0 && rpm3 == 0 && rpm4 == 0) {
    stop_motors();
  } else {
    set_rpm(1, rpm1);
    set_rpm(2, rpm2);
    set_rpm(3, rpm3);
    set_rpm(4, rpm4);
  }
}

void stop_motors() {
  set_rpm(1, 0);
  set_rpm(2, 0);
  set_rpm(3, 0);
  set_rpm(4, 0);
}
