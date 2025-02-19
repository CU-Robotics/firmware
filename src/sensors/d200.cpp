#include "d200.hpp"

D200LD14P::D200LD14P(HardwareSerial *_port, uint8_t _id) {
  port = _port;
  id = _id;
  current_packet = 0;
  port->begin(D200_BAUD);
}

uint8_t D200LD14P::calc_checksum(uint8_t *buf, int len) {
  uint8_t crc8 = 0;
  for (int i = 0; i < len; i++) {
    crc8 = CRC_TABLE[(crc8 ^ *buf++) & 0xff];
  }
  return crc8;
}

void D200LD14P::set_speed(float speed) {
  // convert to deg/s
  uint16_t speed_deg = constrain((uint16_t) (speed * 180.0 / M_PI), MIN_SPEED, MAX_SPEED);

  uint8_t lsb = speed_deg & 0xff;
  uint8_t msb = (speed_deg >> 8) & 0xff;

  uint8_t cmd[D200_CMD_PACKET_LEN] = { 0x54, 0xa2, 0x04, lsb, msb, 0, 0, 0 };
  cmd[D200_CMD_PACKET_LEN - 1] = calc_checksum(cmd, D200_CMD_PACKET_LEN - 1);

  port->write(cmd, D200_CMD_PACKET_LEN);
}

void D200LD14P::start_motor() {
  port->write(D200_START_CMD, D200_CMD_PACKET_LEN);
}

void D200LD14P::stop_motor() {
  port->write(D200_STOP_CMD, D200_CMD_PACKET_LEN);
}

void D200LD14P::read() {
  // consume bytes until we reach a start character (only relevant for startup)
  while (port->available() && port->peek() != D200_START_CHAR) {
    port->read();
  }

  // read packet by packet
  while (port->available() >= D200_DATA_PACKET_LEN) {
    uint8_t start_char = port->read();
    uint8_t frame_char = port->read();
    
    // we either get a data packet or a command packet,
    // determined by the frame character
    int packet_len = frame_char == D200_FRAME_CHAR 
      ? D200_DATA_PACKET_LEN 
      : D200_CMD_PACKET_LEN;
    
    uint8_t buf[packet_len];

    buf[0] = start_char;
    buf[1] = frame_char;
    
    // read remainder of packet into buffer
    port->readBytes(&buf[2], packet_len - 2);

    // we don't care about command packets as long as
    // they are removed from the buffer
    if (frame_char != 0x2c) continue;

    // verify checksum
    uint8_t crc8 = buf[packet_len - 1];
    uint8_t calc_crc8 = calc_checksum(buf, packet_len - 1);

    if (crc8 != calc_crc8) continue;

    // parse packet
    current_packet = (current_packet+1) % D200_NUM_PACKETS_CACHED;
    LidarDataPacketSI *p = &packets[current_packet];

    // (alignments of values from dev manual: https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf)
    uint16_t lidar_speed = (buf[3] << 8) | buf[2];
    uint16_t start_angle = (buf[5] << 8) | buf[4];
    uint16_t end_angle = (buf[43] << 8) | buf[42];
    uint16_t lidar_timestamp = (buf[45] << 8) | buf[44];
    
    // timestamp calibration
    cal.packets_recv++;

    if (cal.prev_timestamp > lidar_timestamp) {
      cal.num_wraps++;
    }
    cal.prev_timestamp = lidar_timestamp;

    // get continuous lidar time by summing wraps
    int lidar_timestamp_cont = cal.num_wraps * D200_TIMESTAMP_WRAP_LIMIT + lidar_timestamp;

    if (cal.packets_recv <= cal.max_calibration_packets) {
      int teensy_timestamp = millis();
      cal.timestamp_delta_sum += (teensy_timestamp - lidar_timestamp_cont);
    } else {
      int lidar_timestamp_converted = lidar_timestamp_cont + cal.timestamp_delta_sum / cal.max_calibration_packets;
      p->timestamp = (float) lidar_timestamp_converted / 1000.0; // ms (wraps after 30k) -> s
    }

    // convert measurements to SI. only write to packet if all calibrations are complete
    if (cal.packets_recv > cal.max_calibration_packets) {
      p->lidar_speed = (float) lidar_speed * M_PI / 180.0; // deg/s -> rad/s
      p->start_angle = ((float) (start_angle % 36000) / 100.0) * M_PI / 180.0; // 0.01 deg -> rad
      p->end_angle = ((float) (end_angle % 36000) / 100.0) * M_PI / 180.0; // 0.01 deg -> rad
      // Serial.printf("lidar: %f, %f, %d\n", p->start_angle,  p->end_angle, current_packet);
      for (int i = 0; i < D200_POINTS_PER_PACKET; i++) {
        // points start at byte 6, each point is 3 bytes
        int base = 6 + i * 3;
        uint16_t distance = (buf[base + 1] << 8) | buf[base];

        // convert measurements to SI
        p->points[i].distance = (float) distance / 1000.0; // mm -> m
        p->points[i].intensity = buf[base + 2]; // units are ambiguous (not documented)
      }
    }
  }
}

void D200LD14P::flush_packet_buffer() {
  LidarDataPacketSI zero_packet = {};
  
  for (int i = 0; i < D200_NUM_PACKETS_CACHED; i++) {
    packets[i] = zero_packet;
  }
}

uint32_t D200LD14P::bitcast_float(float f32) {
  union {
    uint32_t u32;
    float f32;
  } caster;

  caster.f32 = f32;
  return caster.u32;
}

void D200LD14P::export_data(uint8_t bytes[D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE]) {
  // write each packet into byte array
  for (int i = 0; i < D200_NUM_PACKETS_CACHED; i++) {
    int offset = i * D200_PAYLOAD_SIZE;
    LidarDataPacketSI packet = packets[i];

    uint32_t lidar_speed = bitcast_float(packet.lidar_speed);
    uint32_t start_angle = bitcast_float(packet.start_angle);
    uint32_t end_angle = bitcast_float(packet.end_angle);
    uint32_t timestamp = bitcast_float(packet.timestamp);

    bytes[offset + 0] = id;
    
    bytes[offset + 1] = lidar_speed & 0xff;
    bytes[offset + 2] = (lidar_speed >> 8) & 0xff;
    bytes[offset + 3] = (lidar_speed >> 16) & 0xff;
    bytes[offset + 4] = (lidar_speed >> 24) & 0xff;

    bytes[offset + 5] = start_angle & 0xff;
    bytes[offset + 6] = (start_angle >> 8) & 0xff;
    bytes[offset + 7] = (start_angle >> 16) & 0xff;
    bytes[offset + 8] = (start_angle >> 24) & 0xff;

    for (int j = 0; j < D200_POINTS_PER_PACKET; j++) {
      int point_offset = offset + 9 + j * 5;
      uint32_t distance = bitcast_float(packet.points[j].distance);

      bytes[point_offset + 0] = distance & 0xff;
      bytes[point_offset + 1] = (distance >> 8) & 0xff;
      bytes[point_offset + 2] = (distance >> 16) & 0xff;
      bytes[point_offset + 3] = (distance >> 24) & 0xff;

      bytes[point_offset + 4] = packet.points[j].intensity;
    }

    bytes[offset + 69] = end_angle & 0xff;
    bytes[offset + 70] = (end_angle >> 8) & 0xff;
    bytes[offset + 71] = (end_angle >> 16) & 0xff;
    bytes[offset + 72] = (end_angle >> 24) & 0xff;
    
    bytes[offset + 73] = timestamp & 0xff;
    bytes[offset + 74] = (timestamp >> 8) & 0xff;
    bytes[offset + 75] = (timestamp >> 16) & 0xff;
    bytes[offset + 76] = (timestamp >> 24) & 0xff;
  }
}

void D200LD14P::print_latest_packet() {
  LidarDataPacketSI p = get_latest_packet();
  Serial.println("==D200LD14P PACKET==");
  Serial.print("LiDAR speed: ");
  Serial.println(p.lidar_speed);
  Serial.print("start angle: ");
  Serial.println(p.start_angle);
  Serial.println("measurement data: ...");
  Serial.print("end angle: ");
  Serial.println(p.end_angle);
  Serial.print("timestamp: ");
  Serial.println(p.timestamp);
}