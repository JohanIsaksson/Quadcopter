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
  start = 0;
  end = 0;
  size = 0;
}

//----------------------------------------------------------------------------
void Buffer::PushFront(uint8_t data)
{
  start = mapIndex(start - 1);
  buffer[start] = data;
}

//----------------------------------------------------------------------------
void Buffer::PushBack(uint8_t data)
{
  buffer[end] = data;
  end = mapIndex(end + 1);
}

//----------------------------------------------------------------------------
uint8_t Buffer::PopFront()
{
  uint8_t r = buffer[start];
  start = mapIndex(start + 1);
  return r;
}

//----------------------------------------------------------------------------
uint8_t Buffer::PopBack()
{
  end = mapIndex(end - 1);
  return buffer[end];
}

//----------------------------------------------------------------------------
int Buffer::Size()
{
  if (start > end)
  {
    size = maxSize - (start -end);
  }
  else
  {
    size = end - start;
  }
}

//----------------------------------------------------------------------------
int Buffer::StartPos()
{
  return start;
}

//----------------------------------------------------------------------------
int Buffer::EndPos()
{
  return end;
}

//----------------------------------------------------------------------------
uint8_t Buffer::At(int pos)
{
  return buffer[mapIndex(start + pos)];
}

//----------------------------------------------------------------------------
int Buffer::mapIndex(int index)
{
  // TODO: Add overflow warning
  return ((index + maxSize) % maxSize);
}
