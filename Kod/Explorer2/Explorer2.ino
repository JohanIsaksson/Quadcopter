
#pragma GCC optimize("-O3")

#include "explorer.hpp"

Explorer quad;

void setup()
{
  SerialUSB.begin(115200);
  while(!SerialUSB.available())
  {
    
  }
  SerialUSB.println("Initializing Quad...");
  quad.Setup();
  
}

void loop()
{
  quad.Update();
}
