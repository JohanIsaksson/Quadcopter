
#include "gyro.h"


//INTERRUPT DETECTION ROUTINE
void dmpDataReady() {
    mpuInterrupt = true;
}


void init_gyro(gyro* g){
	// join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

 	// initialize device
  g->mpu.initialize();

  g->dmpReady = false;


  // load and configure the DMP
  g->devStatus = g->mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  g->mpu.setXGyroOffset(220);
  g->mpu.setYGyroOffset(76);
  g->mpu.setZGyroOffset(-85);
  g->mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (g->devStatus == 0) {
    // turn on the DMP, now that it's ready
    g->mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    g->mpuIntStatus = g->mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    g->dmpReady = true;

    // get expected DMP packet size for later comparison
    g->packetSize = g->mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
  }
}


bool read_gyro(gyro* g){
	if (!mpuInterrupt && g->fifoCount < g->packetSize){
		return false;
	}

	//reset interrupt
	mpuInterrupt = false;
  g->mpuIntStatus = g->mpu.getIntStatus();

 	// get current FIFO count
  g->fifoCount = g->mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((g->mpuIntStatus & 0x10) || g->fifoCount == 1024) {
      // reset so we can continue cleanly
      g->mpu.resetFIFO();

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (g->mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (g->fifoCount < g->packetSize) g->fifoCount = g->mpu.getFIFOCount();

    // read a packet from FIFO
    g->mpu.getFIFOBytes(g->fifoBuffer, g->packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    g->fifoCount -= g->packetSize;

    // display Euler angles in degrees
    g->mpu.dmpGetQuaternion(&g->q, g->fifoBuffer);
    g->mpu.dmpGetGravity(&g->gravity, &g->q);
    g->mpu.dmpGetYawPitchRoll(g->ypr, &g->q, &g->gravity);

    //convert to degrees
    for (int i = 0; i < 3; i++){
    	g->ypr[i] = g->ypr[i] * 180/M_PI;
    }
    


 	}
 	return true;
}