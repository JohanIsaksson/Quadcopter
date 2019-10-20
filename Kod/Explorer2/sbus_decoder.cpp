#include "sbus_decoder.hpp"
#include <Arduino.h>

SbusDecoder::SbusDecoder()
{
  channels[0] = 1500;
  channels[1] = 1500;
  channels[2] = 1000;
  channels[3] = 1500;
  channels[4] = 1000;
  channels[5] = 1000;
  channels[6] = 1000;
  channels[7] = 1000;
  channels[8] = 1000;
  channels[9] = 1000;
  channels[10] = 1000;
  channels[11] = 1000;
  channels[12] = 1000;
  channels[13] = 1000;
  channels[14] = 1000;
  channels[15] = 1000;
  channels[16] = 1000;
  channels[17] = 1000;

  lastReceived = 0;
}

void SbusDecoder::Update()
{
  while (Serial1.available())
  {
    // Read Sbus data
    buffer.PushBack(Serial1.read());
  }

  if (buffer.Size() >= SBUS_PACKET_SIZE)
  {
    for (int i = 0; i < buffer.Size() - SBUS_PACKET_SIZE; i++)
    {
      if (buffer.At(i) == SBUS_START_BYTE &&
          buffer.At(i + SBUS_PACKET_SIZE) == SBUS_END_BYTE)
      {
        // Complete frame found
        lastReceived = micros();

        // Remove any heading bytes from buffer
        for(int j = 0; j < i; j++)
        {
          buffer.PopFront();
        }

        // Convert sbus frame to ealisy read values
        SbusConversionUnion packet;
        for(int j = 0; j < SBUS_PACKET_SIZE; j++)
        {
          packet.bytes[j] = buffer.PopFront();
        }

        // Map channel values to [1000, 2000]
        channels[0] = Map((uint16_t)packet.frame.channel0);
        channels[1] = Map((uint16_t)packet.frame.channel1);
        channels[2] = Map((uint16_t)packet.frame.channel2);
        channels[3] = Map((uint16_t)packet.frame.channel3);
        channels[4] = Map((uint16_t)packet.frame.channel4);
        channels[5] = Map((uint16_t)packet.frame.channel5);
        channels[6] = Map((uint16_t)packet.frame.channel6);
        channels[7] = Map((uint16_t)packet.frame.channel7);
        channels[8] = Map((uint16_t)packet.frame.channel8);
        channels[9] = Map((uint16_t)packet.frame.channel9);
        channels[10] = Map((uint16_t)packet.frame.channel10);
        channels[11] = Map((uint16_t)packet.frame.channel11);
        channels[12] = Map((uint16_t)packet.frame.channel12);
        channels[13] = Map((uint16_t)packet.frame.channel13);
        channels[14] = Map((uint16_t)packet.frame.channel14);
        channels[15] = Map((uint16_t)packet.frame.channel15);

        channels[16] = ((uint16_t)packet.frame.flags & 0x01) * 1000 + 1000;
        channels[17] = (((uint16_t)packet.frame.flags & 0x02) >> 1) * 1000 + 1000;

        // Flags
        lostFrame = packet.frame.flags & 0x04;
        failSafe = packet.frame.flags & 0x08;

        break;
      }
    }
  }

/*
    uint8_t sbus_packet[25];
  uint16_t channels[16];
  if (Serial1.available() >= 25)
  {

    // Find start of packet 
    // Start byte = 0x0F
    // End byte = 0x00


    //SerialPort.println(Serial1.available());
    if (Serial1.available() == 25)
    {
      for (int i = 0; i < 25; i++)
      {
        sbus_packet[i] = Serial1.read();
      }

      // Verify packet
      if ((sbus_packet[0] == 0x0F))// && (sbus_packet[24] == 0x00))
      {
        // Read channels
        
        // Channel 1
        receiver_input_channel_1 = (sbus_packet[2] & 0x07);
        receiver_input_channel_1 <<= 8;
        receiver_input_channel_1 += sbus_packet[1];
        receiver_input_channel_1 = map(receiver_input_channel_1, 192, 1792, 1000, 2000);

        // Channel 2
        receiver_input_channel_2 = (sbus_packet[3] & 0x3F);
        receiver_input_channel_2 <<= 5;
        receiver_input_channel_2 += ((sbus_packet[2] & 0xF7) >> 3);
        receiver_input_channel_2 = map(receiver_input_channel_2, 192, 1792, 1000, 2000);

        // Channel 3
        receiver_input_channel_3 = (sbus_packet[5] & 0x01);
        receiver_input_channel_3 <<= 8;
        receiver_input_channel_3 += sbus_packet[4];
        receiver_input_channel_3 <<= 2;
        receiver_input_channel_3 += ((sbus_packet[3] & 0xC0) >> 6);
        receiver_input_channel_3 = map(receiver_input_channel_3, 192, 1792, 1000, 2000);

        // Channel 4
        receiver_input_channel_4 = (sbus_packet[6] & 0x0F);
        receiver_input_channel_4 <<= 7;
        receiver_input_channel_4 += ((sbus_packet[5] & 0xFE) >> 1);
        receiver_input_channel_4 = map(receiver_input_channel_4, 192, 1792, 1000, 2000);

        // Channel 5
        receiver_input_channel_5 = (sbus_packet[7] & 0x7F);
        receiver_input_channel_5 <<= 4;
        receiver_input_channel_5 += ((sbus_packet[6] & 0xF0) >> 4) ;
        receiver_input_channel_5 = map(receiver_input_channel_5, 192, 1792, 1000, 2000);

        // Channel 6
        receiver_input_channel_6 = (sbus_packet[9] & 0x03);
        receiver_input_channel_6 <<= 8;
        receiver_input_channel_6 += sbus_packet[8];
        receiver_input_channel_6 <<= 1;
        receiver_input_channel_6 += ((sbus_packet[7] & 0x80) >> 7);
        receiver_input_channel_6 = map(receiver_input_channel_6, 192, 1792, 1000, 2000);

        // Failsafe
        failSafe = (sbus_packet[23] & 0x10) ? true : false;
      }
    }
    else
    {
      // Remove bad data
      while (Serial1.available() > 0)
      {
        Serial1.read();
      }
    }
  }
  */
}

uint16_t SbusDecoder::GetChannelData(int channel)
{
  return channels[channel];
}

uint32_t SbusDecoder::LastReceived()
{
  return lastReceived;
}

uint16_t SbusDecoder::Map(uint16_t x)
{
  return (((5 * x) / 8) + 880);
}