#ifndef REF_SYSTEM
#define REF_SYSTEM

const byte START_OF_FRAME = 0xA5;

uint64_t decode_uint(int, int, byte[]);

typedef enum Stage: uint8_t {
  PreComp = 0,
  SetupPeriod = 1,
  Init = 2,
  Countdown = 3,
  CompOngoing = 4,
  CalcResults = 5
} Stage;

typedef enum GameStatus: uint8_t {
  Draw = 0,
  RedWin = 1,
  BlueWin = 2
} GameStatus;

typedef struct {
  uint16_t barrel_cooling_rate; // /s
  uint16_t barrel_heat_limit;
  uint16_t launch_speed_limit; //m/s
  uint16_t barrel_heat;
} RobotStatus;

typedef struct {
  uint16_t cmd_id;

  // 0x0001: competition status
  Stage stage;
  uint16_t remaining_time;
  uint64_t unix_time;

  // 0x0002: competition result
  GameStatus winner;

  // 0x0201: performance system data
  uint8_t bot_id;
  uint8_t level;
  uint16_t hp;
  uint16_t max_hp;
  uint16_t bot_power_limit;
  uint8_t gimbal_port_out_volts;
  uint8_t chassis_port_out_volts;
  uint8_t shooter_port_out_volts;

  // 0x0202: power heat data
  uint16_t pmm_chassis_port_out_volts;
  uint16_t pmm_chassis_port_out_current;
  uint32_t chassis_power;
  uint16_t buffer_energy;

  // 0x0201 & 0x0202
  RobotStatus bot_1_17mm;
  RobotStatus bot_2_17mm;
  RobotStatus bot_1_42mm;

  // 0x0204: buff data
  uint8_t hp_recovery_buff;
  uint8_t barrel_cool_rate;
  uint8_t defense_buff;
  uint8_t attack_buff;

  // 0x0301: interaction data

} RefSystemPacket;

#endif