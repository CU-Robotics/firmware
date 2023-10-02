#include "DR16.hpp"

DR16::DR16() {}

void DR16::Init()
{
	// start Serial5 HardwareSerial with 1
	Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
	// await any active writing and clear the buffer
	Serial5.flush();
	Serial5.clear();

	// init all input buffers to 0
	for (int i = 0; i < DR16_PACKET_SIZE; i++)
	{
		m_inputRaw[i] = 0;
	}

	for (int i = 0; i < DR16_INPUT_VALUE_COUNT; i++)
	{
		m_input[i] = 0;
	}
}

void DR16::Read()
{
	// each channel is 11 bits, minus the switches
	uint16_t c0{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, wh{ 0 };
	uint8_t s1{ 0 }, s2{ 0 };

	// verify that the receiver is ready to be read
	// since each packet it sends is 18 bytes, verify that there are exactly 8 bytes in the buffer

	// clear if there are more than 18 bytes, i.e. we missed a previous packet
	if (Serial5.available() > DR16_PACKET_SIZE)
	{
		Serial5.clear();
		return;
	}

	// dont read if there are less than 18 bytes, i.e. we caught the packet as it was being written
	if (Serial5.available() < DR16_PACKET_SIZE)
	{
		return;
	}

	// issue read command, fills m_inputRaw with 18 bytes
	Serial5.readBytes(m_inputRaw, DR16_PACKET_SIZE);

	// set channel values, since each channel is packed within each other, and are 11 bits long
	// some bit shifting is required
	c0 = ((m_inputRaw[1] & 0x07) << 8) 	| m_inputRaw[0];
	c1 = ((m_inputRaw[2] & 0x3f) << 5) 	| ((m_inputRaw[1] & 0xf8) >> 3);
	c2 = ((m_inputRaw[4] & 0x01) << 10)	| ((m_inputRaw[3] & 0xff) << 2) | ((m_inputRaw[2] & 0xc0) >> 6);
	c3 = ((m_inputRaw[5] & 0x0f) << 7) 	| ((m_inputRaw[4] & 0xfe) >> 1);
	wh = ((m_inputRaw[17] & 0x7) << 8) 	| m_inputRaw[16];
	s1 = (m_inputRaw[5] & 0x30) >> 4;
	s2 = (m_inputRaw[5] & 0xc0) >> 6;

  // set these split values into the seperated raw input array
  m_inputRawSeperated[0] = c0;
  m_inputRawSeperated[1] = c1;
  m_inputRawSeperated[2] = c2;
  m_inputRawSeperated[3] = c3;
  m_inputRawSeperated[4] = wh;
  m_inputRawSeperated[5] = s1;
  m_inputRawSeperated[6] = s2;

  // simple safety check
	// assign formated data (within ranges of [-1,1]) to the true input buffer
#ifdef ENABLE_VALUE_CHECK_SAFETY
  if (IsDataValid())
  {
#endif
    // joy sticks
    m_input[0] = bounded_map(c0, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
    m_input[1] = bounded_map(c1, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
    m_input[2] = bounded_map(c2, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
    m_input[3] = bounded_map(c3, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;

    // wheel
    m_input[4] = bounded_map(wh, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
    
    // switches
    m_input[5] = (float)s1;
    m_input[6] = (float)s2;
#ifdef ENABLE_VALUE_CHECK_SAFETY
  }
  else 
  {
#endif
    Zero();
    Serial5.clear();
    Serial.println("\n\n\nDiscarded bad packet\n\n\n");
    Serial.print("Received: "); PrintRaw();
    Serial.print("Expected: "); Print();
#ifdef ENABLE_VALUE_CHECK_SAFETY
  }
#endif

  Serial.printf("%.4d (%.3f)\t%.4d (%.3f)\t%.4d (%.3f)\t%.4d (%.3f)\t%.4d (%.3f)\t%.4d\t%.4d\n", c0, m_input[0], c1, m_input[1], c2, m_input[2], c3, m_input[3], wh, m_input[4], s1, s2);

	// Print();
}

void DR16::Zero()
{
  // zero input buffer
  for (int i = 0; i < DR16_INPUT_VALUE_COUNT; i++)
  {
    m_input[i] = 0;
  }

  // set switches to a specific value
  m_input[5] = 1;
  m_input[6] = 1;
}

float* DR16::GetInput()
{
	return m_input;
}

void DR16::Print()
{
	Serial.printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", m_input[0], m_input[1], m_input[2], m_input[3], m_input[4], m_input[5], m_input[6]);
}

void DR16::PrintRaw()
{
	for (int i = 0; i < DR16_PACKET_SIZE; i++)
		Serial.printf("%.2x\t", m_inputRaw[i]);
	Serial.println();
}

float DR16::bounded_map(int value, int in_low, int in_high, int out_low, int out_high)
{
	// this is derived from sthe arduino map() function
	value = max(min(value, in_high), in_low);
	
	return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

bool DR16::IsDataValid()
{
  // go through all values in raw seperated input and compare them against maximum and minimum values
  // the - 2 is to exclude switch values
  for (int i = 0; i < DR16_INPUT_VALUE_COUNT - 2; i++)
  {
    if (m_inputRawSeperated[i] < DR16_CONTROLLER_INPUT_LOW || m_inputRawSeperated[i] > DR16_CONTROLLER_INPUT_HIGH)
      return false;
  }

  return true;
}
