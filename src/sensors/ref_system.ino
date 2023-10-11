#include "ref_system.hpp"

void setup() {
  Serial2.begin(115200);
}

uint64_t decode_uint(int offset, int size, byte data[]) {
  if (size == 1) {
    return data[offset];
  } else {
    uint64_t result = 0;
    for (int i = 0; i < min(size, 8); i++) {
      result |= data[i] << i * 8;
    }

    return result;
  }
}

#define DECODE_SHORT(offset, data) static_cast<uint16_t>(decode_uint(offset, 2, data))

RefSystemPacket packet;

void loop() {
  byte temp[4] = {0};

  // https://rm-static.djicdn.com/tem/71710/RoboMaster%20Referee%20System%20Serial%20Port%20Protocol%20Appendix%20V1.5%EF%BC%8820230717%EF%BC%89.pdf
  if (Serial2.available() > 1) {
    int sof = Serial2.read();

    if (sof == START_OF_FRAME) {
      if (Serial2.readBytes(temp, 4) < 4) {
        return;
      }

      uint16_t data_length = DECODE_SHORT(0, temp);

      // TODO: check CRC8
      uint8_t seq = temp[2];
      uint8_t crc8 = temp[3];

      if (Serial2.readBytes(temp, 2) < 2) {
        return;
      }
    
      uint16_t cmd_id = DECODE_SHORT(0, temp);

      byte data[data_length];
      if (Serial2.readBytes(data, data_length) < data_length) {
        return;
      }

      if (Serial2.readBytes(temp, 2) < 2) {
        return;
      }

      // TODO: check crc16
      uint16_t crc16 = DECODE_SHORT(0, temp);

      // packet types
      packet.cmd_id = cmd_id;
      switch (cmd_id) {
        case 0x0001:

          packet.stage = static_cast<Stage>(data[0]);
          packet.remaining_time = DECODE_SHORT(1, data);
          packet.unix_time = static_cast<uint64_t>(decode_uint(3, 8, data));

          break;
        case 0x0002:

          packet.winner = static_cast<GameStatus>(data[0]);

          break;
        case 0x0201:

          packet.bot_id = data[0];
          packet.level = data[1];
          packet.hp = DECODE_SHORT(2, data);
          packet.max_hp = DECODE_SHORT(4, data);
          
          packet.bot_1_17mm.barrel_cooling_rate = DECODE_SHORT(6, data);
          packet.bot_1_17mm.barrel_heat_limit = DECODE_SHORT(8, data);
          packet.bot_1_17mm.launch_speed_limit = DECODE_SHORT(10, data);

          packet.bot_2_17mm.barrel_cooling_rate = DECODE_SHORT(12, data);
          packet.bot_2_17mm.barrel_heat_limit = DECODE_SHORT(14, data);
          packet.bot_2_17mm.launch_speed_limit = DECODE_SHORT(16, data);
          
          packet.bot_1_42mm.barrel_cooling_rate = DECODE_SHORT(18, data);
          packet.bot_1_42mm.barrel_heat_limit = DECODE_SHORT(20, data);
          packet.bot_1_42mm.launch_speed_limit = DECODE_SHORT(22, data);

          packet.bot_power_limit = DECODE_SHORT(24, data);

          // bit flags indicate output volts
          {
            uint8_t out_volts = data[26];
            packet.gimbal_port_out_volts = 0x1 & out_volts ? 24 : 0;
            packet.chassis_port_out_volts = 0x2 & out_volts ? 24 : 0;
            packet.shooter_port_out_volts = 0x4 & out_volts ? 24 : 0;
          }

          break;
        case 0x0202:

          packet.pmm_chassis_port_out_volts = DECODE_SHORT(0, data);
          packet.pmm_chassis_port_out_current = DECODE_SHORT(2, data);
          packet.chassis_power = static_cast<uint32_t>(decode_uint(4, 4, data));
          packet.buffer_energy = DECODE_SHORT(8, data);

          packet.bot_1_17mm.barrel_heat = DECODE_SHORT(10, data);
          packet.bot_2_17mm.barrel_heat = DECODE_SHORT(12, data);
          packet.bot_1_42mm.barrel_heat = DECODE_SHORT(14, data);

          break;
        case 0x0204:

          packet.hp_recovery_buff = data[0];
          packet.barrel_cool_rate = data[1];
          packet.defense_buff = data[2];
          packet.attack_buff = DECODE_SHORT(3, data);

          break;
        case 0x0301:



          break;
      }

#define REF_SYSTEM_DEBUG
#ifdef REF_SYSTEM_DEBUG
      // print packet data here
#endif
    }
  }
}

#undef DECODE_SHORT
