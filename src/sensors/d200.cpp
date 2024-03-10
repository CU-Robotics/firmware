#include "d200.hpp"

D200LD14P::D200LD14P() {
  state.buffer_idx = 0;
  LIDAR_SERIAL.begin(LIDAR_BAUD);
  start();
}

LidarDataPacket D200LD14P::get_latest_packet() {
  return latest_packet;
}

uint8_t D200LD14P::calc_checksum(uint8_t *buf, int len) {
  uint8_t crc8 = 0;
  for (int i = 0; i < len; i++) {
    crc8 = CRC_TABLE[(crc8 ^ *buf++) & 0xff];
  }
  return crc8;
}

void D200LD14P::set_speed(uint16_t speed) {
  uint8_t lsb = speed & 0xff;
  uint8_t msb = (speed >> 8) & 0xff;

  uint8_t cmd[CMD_LEN] = { 0x54, 0xa2, 0x04, lsb, msb, 0, 0, 0 };
  cmd[CMD_LEN - 1] = calc_checksum(cmd, CMD_LEN - 1);

  LIDAR_SERIAL.write(cmd, CMD_LEN);
}

void D200LD14P::start() {
  LIDAR_SERIAL.write(START_CMD, CMD_LEN);
}

void D200LD14P::stop() {
  LIDAR_SERIAL.write(STOP_CMD, CMD_LEN);
}

void D200LD14P::read() {
  while (LIDAR_SERIAL.available()) {
    // if the buffer is full, parse the packet
    if (state.buffer_idx == PACKET_SIZE) {
      // verify checksum
      uint8_t crc8 = state.buffer[PACKET_SIZE - 1];
      uint8_t calc_crc8 = calc_checksum(state.buffer, PACKET_SIZE - 1);

      if (crc8 == calc_crc8) {
        // data values are little-endian
        uint8_t *b = &state.buffer[0];
        LidarDataPacket *p = &state.latest_packet;
        
        p->lidar_speed = (b[3] << 8) | b[2];
        p->start_angle = (b[5] << 8) | b[4];

        for (int i = 0; i < POINTS_PER_PACKET /* 12 */; i++) {
          int base = 6 + i * 3;
          p->points[i].distance = (b[base + 1] << 8) | b[base];
          p->points[i].distance = b[base + 2];
        }

        p->end_angle = (b[43] << 8) | b[42];
        p->timestamp = (b[45] << 8) | b[44];
      }

      // reset the buffer index to start reading the next packet
      state.buffer_idx = 0;
    }

    // if we are reading the first two bytes of a packet, check for start and frame characters
    // otherwise, if the buffer is not full, keep filling it
    int next_byte = LIDAR_SERIAL.read();
    switch (state.buffer_idx) {
      case 0:
        if (next_byte == START_CHAR) {
          state.buffer[state.buffer_idx++] = next_byte;
        }
        break;
      case 1:
        if (next_byte == FRAME_CHAR) {
          state.buffer[state.buffer_idx++] = next_byte;
        } else {
          state.buffer_idx = 0;
        }
        break;
      default:
        state.buffer[state.buffer_idx++] = next_byte;
        break;
    }
  }
}

void D200LD14P::print() {
  Serial.println("==D200LD14P PACKET==");
  Serial.print("LiDAR speed: ");
  Serial.println(latest_packet.lidar_speed);
  Serial.print("start angle: ");
  Serial.println(latest_packet.start_angle);
  Serial.println("measurement data: ...");
  Serial.print("end angle: ");
  Serial.println(latest_packet.end_angle);
  Serial.print("timestamp: ");
  Serial.println(latest_packet.timestamp);
}