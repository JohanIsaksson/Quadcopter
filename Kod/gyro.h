#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif


struct gyro{
	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float ypr_offsets[3]; // offsets

	MPU6050 mpu; // gyro

};
typedef struct gyro gyro;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

