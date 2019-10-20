#ifndef DECODER_H
#define DECODER_H

#include "buffer.hpp"

#define SBUS_PACKET_SIZE 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00

typedef struct SbusFrame
{
  uint8_t start;
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  unsigned int channel0 : 11;
  unsigned int channel1 : 11;
  unsigned int channel2 : 11;
  unsigned int channel3 : 11;
  unsigned int channel4 : 11;
  unsigned int channel5 : 11;
  unsigned int channel6 : 11;
  unsigned int channel7 : 11;
  unsigned int channel8 : 11;
  unsigned int channel9 : 11;
  unsigned int channel10 : 11;
  unsigned int channel11 : 11;
  unsigned int channel12 : 11;
  unsigned int channel13 : 11;
  unsigned int channel14 : 11;
  unsigned int channel15 : 11;
  uint8_t flags;
  uint8_t end;
} __attribute__((__packed__)) SbusFrame;
// Creds to beta flight for introducing me __packed__ attribute

typedef union SbusConversionUnion
{
  uint8_t bytes[SBUS_PACKET_SIZE];
  SbusFrame frame;
} SbusConversionUnion;

class SbusDecoder
{

public:
  SbusDecoder();
  void Update();

  uint16_t GetChannelData(int channel);
  uint32_t LastReceived();


private:
  Buffer buffer{};
  uint16_t channels[18];

  uint64_t lastReceived;
  bool failSafe;
  bool lostFrame;

  uint16_t Map(uint16_t x);

};


#endif /* DECODER_H */