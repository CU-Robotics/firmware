#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

// FlexCAN_T4 library and supporting headers
#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>           // issues a warning of a nullptr reference
#include <isotp_server.h>    
#include <kinetis_flexcan.h>

// C620 Brushless DC Motor Speed Controller
// Documentation: https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf 

// FlexCAN_T4
// Documentation: https://github.com/tonton81/FlexCAN_T4

constexpr uint16_t CAN_1 = 0;             // CAN 1 (indexable value)
constexpr uint16_t CAN_2 = 1;             // CAN 2 (indexable value)

constexpr uint16_t NUM_CANS = 2;          // 2 cans per robot
constexpr uint16_t NUM_MOTORS = 8;        // 8 motors per can
constexpr uint16_t NUM_MESSAGE_IDS = 3;   // 3 messages per can: 0x200, 0x1ff, 0x2ff

constexpr uint16_t CAN_MESSAGE_SIZE = 8;  // 8 uint8_t's per message buffer

/// @brief Simple enum to specify types of values able to be read from motors
enum MotorAttribute
{
  ANGLE, SPEED, TORQUE, TEMP
};

/// @brief Manages both CANs on the robot. Able to read from and write to individual or multiple motors on the CANs.
class CAN_Manager
{
public:
  /// @brief Defaulted/ignored constructor. Use Init() to initialize CAN_Manager
  CAN_Manager() = default;

  /// @brief Initializes and zeros CANs
  void Init();

  /// @brief Reads current CAN input and fills the input array array accordingly
  /// @return True or false depending if it actually read anything
  uint8_t Read();

  /// @brief Writes current values from output array to the CANs
  /// @return True or false depending if the operation was successful
  /// @note Does issue a Write command to the CANs
  uint8_t Write();

public:
  /// @brief Sets the buffer values in output array to 0
  /// @note Does not issue a Write command to the CANs
  void ZeroOutput();

  /// @brief Sets output array to 0 and writes 0s to the CANs
  /// @note Does issue a Write command to the CANs
  void ZeroCANs();

  /// @brief Sets the output array message for the corresponding motor
  /// @note Does not issue a Write command to the CANs
  /// @param canID ID of the CAN which the motor is on, expects indexable ID value
  /// @param motorID ID of the individual motor, expects indexable ID value
  /// @param value A value in the range of []
  void WriteMotor(uint16_t canID, uint16_t motorID, int32_t value);

  /// @brief Reads and returns value from input array of specified motor
  /// @param canID ID of the CAN which the motor is on, expects indexable ID value
  /// @param motorID ID of the individual motor, expects indexable ID value
  /// @param valueType MotorAttribute enum for what type of value requested
  /// @return Requested value
  uint16_t GetMotorAttribute(uint16_t canID, uint16_t motorID, MotorAttribute valueType);

  /// @brief Print off all values associated with specified motor
  /// @param canID ID of the CAN which the motor is on, expects indexable ID value
  /// @param motorID ID of the individual motor, expects indexable ID value
  /// @param showFullHex Show the entire buffer in hex, rather than decimal values (defaults to false)
  void PrintMotor(uint16_t canID, uint16_t motorID, bool showFullHex = false);

  /// @brief Print off all motors associated with specified CAN
  /// @param canID ID of the CAN which the motor is on, expects indexable ID value
  /// @param showFullHex Show the entire buffer in hex, rather than decimal values (defaults to false)
  void PrintCAN(uint16_t canID, bool showFullHex = false);

  void PrintOutput();

private:
  /// @brief CAN 1 object
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can1;
  /// @brief CAN 2 object
  FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> m_can2;

  /// @brief Output array in the format of CAN_message_t's
  uint8_t m_output[NUM_CANS][NUM_MESSAGE_IDS][CAN_MESSAGE_SIZE];
  /// @brief Input array in the format of uint8_t's
  uint8_t m_input[NUM_CANS][NUM_MOTORS][CAN_MESSAGE_SIZE];
};

/// @brief Returns a 2-byte value given 2 1-byte values
/// @param highByte higher order byte
/// @param lowByte lower order byte
/// @return Constructed 2-byte value in form [high, low]
inline uint16_t combineBytes(uint8_t high, uint8_t low)
{
  uint16_t result = 0;
  return ((result | high) << 8) | low;
}

#endif // CAN_MANAGER_HPP

CAN_Manager can;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  can.Init();
}

void loop() {
  Serial.println("loop begin");
  delay(10);

  Serial.println("Pre read");
  while (can.Read()) { Serial.print("r"); }
  Serial.println();
  Serial.println("Post read");

  can.WriteMotor(0, 0, 0);

  can.PrintOutput();

  can.Write();
}

void CAN_Manager::Init()
{
    // initialize CAN 1
    m_can1.begin();
    m_can1.setBaudRate(1000000);
    m_can1.enableFIFO(true);

    // initialize CAN 2
    m_can2.begin();
    m_can2.setBaudRate(1000000);
    m_can2.enableFIFO(true);

    // zero CANs just in case
    ZeroCANs();
}

uint8_t CAN_Manager::Read()
{
    // read from CAN 1
    CAN_message_t msg1;
    uint16_t read1;
    
    read1 = m_can1.read(msg1);
    if (read1)
    {
        // isolate the ID part of the message id (0x202 becomes 2)
        int id = msg1.id & 0xf;
        id -= 1; // subtract by 1 to allow for array indexing
        
        // fill appropriate buffer
        for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
            m_input[CAN_1][id][i] = msg1.buf[i];
    }

    // read from CAN 2
    CAN_message_t msg2;
    uint16_t read2;
    
    read2 = m_can2.read(msg2);
    if (read2)
    {
        // isolate the ID part of the message id (0x202 becomes 2)
        int id = msg1.id & 0xf;
        id -= 1; // subtract by 1 to allow for array indexing
        
        // fill appropriate buffer
        for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
            m_input[CAN_2][id][i] = msg2.buf[i];
    }

    // return true if either read was successful
    return read1 || read2;
}

uint8_t CAN_Manager::Write()
{
    CAN_message_t out;
    out.id = 0x200;
    for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
        out.buf[i] = m_output[CAN_1][0][i];
    Serial.print(m_can1.write(out));

    out.id = 0x1ff;
    for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
        out.buf[i] = m_output[CAN_1][1][i];
    Serial.print(m_can1.write(out));

    out.id = 0x200;
    for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
        out.buf[i] = m_output[CAN_2][0][i];
    Serial.print(m_can2.write(out));

    out.id = 0x1ff;
    for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
        out.buf[i] = m_output[CAN_2][1][i];
    Serial.print(m_can2.write(out));

    return 0;
}

void CAN_Manager::ZeroOutput()
{
    // go through the entire m_output array and zero the buffers
    for (int i = 0; i < NUM_CANS; i++)
    {
        for (int j = 0; j < NUM_MESSAGE_IDS; j++)
        {
            for (int k = 0; k < CAN_MESSAGE_SIZE; k++)
            {
                m_output[i][j][k] = 0;
            }
        }
    }
}

void CAN_Manager::ZeroCANs()
{
    // zero all output messages
    ZeroOutput();

    // write all output messages (now all 0s)
    Write();
}

void CAN_Manager::WriteMotor(uint16_t canID, uint16_t motorID, int32_t value)
{
    // find the corresponding message ID based on the motor ID
    // this division will truncate motors: 
    //      [0 - 3]  to index 0 (ID: 0x200)
    //      [4 - 7]  to index 1 (ID: 0x1ff)
    //      [8 - 11] to index 2 (ID: 0x2ff)
    uint8_t messageID = motorID / 4;
    uint8_t mID = motorID % 4;

    // set to buffer indexes (motorID * 2) and (motorID * 2 + 1) corresponding to the
    // expected output by the motors. Explained in motor documentation (page 14-16)

    // set big byte to correct index in buffer
    m_output[canID][messageID][mID * 2]     = (value >> 8) & 0xff;
    // set small byte to correct index in buffer
    m_output[canID][messageID][mID * 2 + 1] = value & 0xff;
}

uint16_t CAN_Manager::GetMotorAttribute(uint16_t canID, uint16_t motorID, MotorAttribute valueType)
{
    // return correct value depending on valueType enum
    switch (valueType)
    {
    case MotorAttribute::ANGLE:
        return combineBytes(m_input[canID][motorID][0], m_input[canID][motorID][1]);
        break;
    case MotorAttribute::SPEED:
        return combineBytes(m_input[canID][motorID][2], m_input[canID][motorID][3]);
        break;
    case MotorAttribute::TORQUE:
        return combineBytes(m_input[canID][motorID][4], m_input[canID][motorID][5]);
        break;
    case MotorAttribute::TEMP:
        return m_input[canID][motorID][6];
        break;

    default:
        return 0;
        break;
    }
}

void CAN_Manager::PrintMotor(uint16_t canID, uint16_t motorID, bool showFullHex)
{
    // strip angle, speed, torque, and temp from input array
    uint16_t angle = combineBytes(m_input[canID][motorID][0], m_input[canID][motorID][0 + 1]);
    uint16_t speed = combineBytes(m_input[canID][motorID][2], m_input[canID][motorID][2 + 1]);
    uint16_t torque = combineBytes(m_input[canID][motorID][4], m_input[canID][motorID][4 + 1]);
    uint16_t temp = m_input[canID][motorID][6];

    // speed and torque are interpreted as signed values. all others are unsigned

    if (showFullHex)
        Serial.printf("CAN: %x\tMotor: %x\t\t%.4x\t%.4x | %.4x\t%.4x | %.4x\t%.4x | %.4x\t%.4x\n", canID + 1, motorID + 1, (angle >> 8) & 0xff, angle & 0xff, (speed >> 8) & 0xff, speed & 0xff, (torque >> 8) & 0xff, torque & 0xff, temp, 0);
    else 
        Serial.printf("CAN: %x\tMotor: %x\t\tAngle: %.4u\tSpeed: %.5d\tTorque: %.5d\tTemp: %.2u\n", canID + 1, motorID + 1, angle, speed, torque, temp);
}

void CAN_Manager::PrintCAN(uint16_t canID, bool showFullHex)
{
    // for all motors, print that motor
    for (int i = 0; i < NUM_MOTORS; i++)
        PrintMotor(canID, i, showFullHex);
}

void CAN_Manager::PrintOutput()
{
  for (int i = 0; i < NUM_CANS; i++)
  {
    Serial.printf("CAN: %x\n", i);
    for (int j = 0; j < NUM_MESSAGE_IDS; j++)
    {
      Serial.printf("Message: %x\t", j);
      for (int k = 0; k < CAN_MESSAGE_SIZE; k++)
      {
        Serial.printf("%x ", m_output[i][j][k]);
      }
      Serial.println();
    }
  }
  Serial.println();
}
