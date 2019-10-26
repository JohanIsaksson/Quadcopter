#include "buffer.hpp"

//----------------------------------------------------------------------------
Buffer::Buffer(int mSize)
{
  if (mSize <= BUFFER_SIZE)
  {
    maxSize = mSize;
  }
  else
  {
    maxSize = BUFFER_SIZE;
  }
  startPos = 0;
  endPos = 0;
  currentSize = 0;
}

//----------------------------------------------------------------------------
void Buffer::PushFront(uint8_t data)
{
  startPos = mapIndex(startPos - 1);
  buffer[startPos] = data;
}

//----------------------------------------------------------------------------
void Buffer::PushBack(uint8_t data)
{
  buffer[endPos] = data;
  endPos = mapIndex(endPos + 1);
}

//----------------------------------------------------------------------------
uint8_t Buffer::PopFront()
{
  uint8_t r = buffer[startPos];
  startPos = mapIndex(startPos + 1);
  return r;
}

//----------------------------------------------------------------------------
uint8_t Buffer::PopBack()
{
  endPos = mapIndex(endPos - 1);
  return buffer[endPos];
}

//----------------------------------------------------------------------------
int Buffer::Size()
{
  if (startPos > endPos)
  {
    currentSize = maxSize - (startPos - endPos);
  }
  else
  {
    currentSize = endPos - startPos;
  }
  return currentSize;
}

//----------------------------------------------------------------------------
int Buffer::MaxSize()
{
  return maxSize;
}

//----------------------------------------------------------------------------
int Buffer::StartPos()
{
  return startPos;
}

//----------------------------------------------------------------------------
int Buffer::EndPos()
{
  return endPos;
}

//----------------------------------------------------------------------------
uint8_t Buffer::At(int pos)
{
  return buffer[mapIndex(startPos + pos)];
}

//----------------------------------------------------------------------------
int Buffer::mapIndex(int index)
{
  // TODO: Add overflow warning
  return ((index + maxSize) % maxSize);
}
