#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>

#define BUFFER_SIZE 128

class Buffer
{
public:
  Buffer(int mSize = BUFFER_SIZE);
  void PushFront(uint8_t data);
  void PushBack(uint8_t data);
  uint8_t PopFront();
  uint8_t PopBack();
  int Size();
  int StartPos();
  int EndPos();
  uint8_t At(int pos);

private:
  int start;
  int end;
  int size;
  int maxSize;
  bool overflow;
  uint8_t buffer[BUFFER_SIZE];

  int mapIndex(int index);
};

#endif /* BUFFER_H */