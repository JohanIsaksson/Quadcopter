#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"

#define SPEED_MIN 1000
#define SPEED_MAX 2000

#define SYSTEM_PERIOD 4000

#define REF_MAX_HORIZON 25.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 120.0

#define MODE_HORIZON 0
#define MODE_ACRO 1

#define FLAG_OTHER 3
#define FLAG_M 4
#define FLAG_OVERHEAD 5


//#define YPR_DATA
//#define COMP_DATA
//#define PID_DATA
//#define MAG_DATA
//#define JAVA_DATA
//#define RADIO_DATA
//#define ESC_DATA

imu im;
Pid pid_roll, pid_pitch, pid_yaw,
    pid_roll_rate, pid_pitch_rate, pid_yaw_rate;


//motors 


int throttle[4]; /* 0 = front left
                    1 = front right
                    2 = back left
                    3 = back right
                  */

//pid
int front;
int left;
int cw;

//output
int count;

//safety stuff
int radio_off_counter;

//time keeping
uint32_t time_diff;
uint32_t time_last;
double timed;

//I2C stuff
uint8_t I2C_cur;

//baro stuff
bool baro_start;


double rad_roll, rad_pitch, rad_yaw;
uint16_t receiver_throttle, receiver_roll, receiver_pitch, receiver_yaw;

//motor control variables
bool motors_on, disable_sticks;
uint32_t us, esc_time, start_time, end_time, esc_t;

//flight mode control
uint8_t flight_mode;
uint8_t c = 0;


double map_d(double x, double in_min, double in_max, double out_min, double out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_motors(){
  uint32_t init_esc_time = micros() + 3000000;
  for(int i = 0; i < 4; i++){
    throttle[i] = SPEED_MIN;
  }

  while (micros() < init_esc_time){ //run for 3s approx

    start_time = micros();
    end_time = start_time + 2000;
    PORTB |= B00001111; // set all inputs to 1

    esc_time = micros();

    while(end_time > esc_time){ 
    	esc_t = esc_time - start_time;                        
      if (esc_t >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_t >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_t >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_t >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }

    PORTB &= B11110000; //for sfety, set all outputs to 0
      
    while(micros() - time_last < 4000);
  }
}

void set_motor_speeds(){
  //front left
  throttle[0] = receiver_throttle + front + left + cw; //+ CALIBRATION_COMP;

  //front right
  throttle[1] = receiver_throttle + front - left - cw; //+ CALIBRATION_COMP;

  //back left
  throttle[2] = receiver_throttle - front + left - cw; //- CALIBRATION_COMP;

  //back right
  throttle[3] = receiver_throttle - front - left + cw; //- CALIBRATION_COMP;

  //check and set speeds
  for (int i = 0; i < 4; i++){
    if (throttle[i] > SPEED_MAX){
      throttle[i] = SPEED_MAX;
    }else if (throttle[i] < SPEED_MIN){
      throttle[i] = SPEED_MIN;
    }        
  }

  esc_time = micros();

  while(end_time > esc_time){ 
    	esc_t = esc_time - start_time;                        
      if (esc_t >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_t >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_t >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_t >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }
  PORTB &= B11110000; //turn all pulses off for safety

  if (flight_mode == MODE_HORIZON){
    while(micros() - time_last < 4000);
    }else{
      while(micros() - time_last < 3000);
    }
}

void set_motor_speeds_min(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  for (int i = 0; i < 4; i++){
      throttle[i] = SPEED_MIN;
    }

  esc_time = micros();

  while(end_time > esc_time){ 
    	esc_t = esc_time - start_time;                        
      if (esc_t >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_t >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_t >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_t >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }
  PORTB &= B11110000; //turn all pulses off for safety

    
  while(micros() - time_last < 4000);
}

void set_motor_speeds_throttle(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  throttle[0] = 1000;
  throttle[1] = receiver_throttle;
  throttle[2] = 1000;
  throttle[3] = 1000;

  esc_time = micros();

  while(end_time > esc_time){ 
    	esc_t = esc_time - start_time;                        
      if (esc_t >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_t >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_t >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_t >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }
  PORTB &= B11110000; //turn all pulses off for safety

    
  while(micros() - time_last < 4000);
}

void setup(){


  Serial.begin(38400);
  Serial.println("Starting up");

	imu_init(&im);

  pid_roll_rate.init();
  pid_roll_rate.set_constants(2.5, 0.2, 0.005);

  pid_roll.init();

  pid_pitch_rate.init();
  pid_roll_rate.set_constants(2.5,0.2,0.005);

  pid_pitch.init();

  pid_yaw_rate.init();
  pid_yaw_rate.set_constants(2.0,0.01,0.0);

  pid_yaw.init();

  front = 0;
  left = 0;
  cw = 0;


  count = 0;

  //wait for esc init;
  Serial.println("Initializing ESCs...");
  //asign motors to pins

  DDRB |= B00001111; //set pins as outputs

  DDRD &= B10111111; //set pin D6 as input - disable I2C

  //initialize escs
  init_motors();
  motors_on = true;
  flight_mode = MODE_ACRO; //--------------------------------------------------------
  receiver_throttle = 1000;
  receiver_roll = 1500;
  receiver_pitch = 1500;
  receiver_yaw = 1500;

  Wire.begin();        // join i2c bus
  I2C_cur = 0;

  time_last = micros();
  Serial.println("Running...");

}

void update_horizon(uint32_t t){

  //update all sensor on the imu
  imu_update_horizon(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;



  //p.K_D_yaw = map_d((double)receiver_input_channel_5, 975.0, 2000.0, 0.0, 0.1);
  //p.K_P_yaw = map_d((double)receiver_input_channel_6, 975.0, 2000.0, 0.0, 1.5);

  //map inputs to angles
  rad_roll = map_d((double)receiver_roll,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_d((double)receiver_pitch,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_yaw = map_d((double)receiver_yaw,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);

  //calculate pids
  timed = (double)t/1000000.0;
  pid_pitch.update(&front, im.ypr[1], rad_pitch, timed, 1.0);
  pid_roll.update(&left, im.ypr[2], rad_roll, timed, 1.0);
  pid_yaw_rate.update(&cw, -im.z_gyr*RAD_TO_DEG, rad_yaw, 1.0, 1.0); //fix time
}

void update_acro(uint32_t t){

  //update only the gyroscope on the imu
  imu_update_acro(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  //map inputs to anglerates
  rad_roll = map_d((double)receiver_roll,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_pitch = map_d((double)receiver_pitch,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_yaw = map_d((double)receiver_yaw,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
  timed = (double)t/1000000.0;
  pid_pitch_rate.update(&front, im.y_gyr*RAD_TO_DEG, rad_pitch, timed, 1.0); 
  pid_roll_rate.update(&left, im.x_gyr*RAD_TO_DEG, rad_roll, timed, -1.0);
  pid_yaw_rate.update(&cw, -im.z_gyr*RAD_TO_DEG, rad_yaw, 1.0, 1.0); //fix time
}

void loop(){

  time_diff = micros() - time_last;
  time_last = micros();

  get_data();
 

  if (motors_on){
    //update according to set flight mode
    if(flight_mode == MODE_HORIZON){
      update_horizon(time_diff);
      //send_data();
    }else if (flight_mode == MODE_ACRO){
      update_acro(time_diff);
    }    
    //apply new speed to motors
    set_motor_speeds();
  }else{
    //keep motors updated
    set_motor_speeds_min();
  }

  
  //delay(200);
  /*
  c++;
  if (c == 8){
    c = 0;
    //Serial.print("$");
    //Serial.println(flight_mode ? "ACROBATIC" : "HORIZON"); //pitch
    //Serial.println(im.altitude);
  }*/
  //Serial.println(p.K_tmp, 8);

  /*Serial.print(motors_on ? "ON" : "OFF");
  Serial.print("\t");
  Serial.print(flight_mode);// ? "ON" : "OFF");
  Serial.print("\t");*/
  /*
  Serial.print(receiver_roll);
  Serial.print("\t");
  Serial.print(receiver_pitch);
  Serial.print("\t");
  Serial.print(receiver_throttle);
  Serial.print("\t");
  Serial.println(receiver_yaw);*/
  
} 



void get_data() {
  Wire.requestFrom(8, 2);    // request 2 bytes from slave device #8
  uint8_t I2C_buffer[2] = {0,0};
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    I2C_buffer[i] = Wire.read(); // receive a byte
    i++;
  }

  I2C_cur = (I2C_buffer[1] >> FLAG_OVERHEAD);

  motors_on = (I2C_buffer[1] & B00010000) >> FLAG_M;

  ///other = (I2C_buffer[1] & B00001000) >> FLAG_OTHER;

  flight_mode = (I2C_buffer[1] & B00000111);

  uint16_t data = I2C_buffer[0];
  data <<= 2;
  data += 1000; 


  switch(I2C_cur) {
    case 1:
      receiver_roll = data;
    break;

    case 2:
      receiver_pitch = data;
    break;

    case 3:
      receiver_throttle = data;
    break;

    case 4:
      receiver_yaw = data;
    break;

    case 5:
      //pid_roll_rate.K_tmp = map_d((double)data, 1000.0, 2000.0, 0.0, 5.0);

    break;

  }
}

void send_data(){
  int8_t a = (int8_t)(im.vertical_acc*32.0);
  int16_t h = (int16_t)(im.altitude*100.0);
  uint8_t hl = h;
  uint8_t hh = h >> 8;

  uint8_t I2C_buffer[3] = {hh, hl, a};
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(I2C_buffer, 3);        // send three bytes
  Wire.endTransmission();    // stop transmitting

  //Serial.println(h);
}




// Motor calibration code
// Add the following code to the beginning of loop()

/*
  time_diff = micros() - time_last;
  time_last = micros();

  get_data();

  if (receiver_throttle > 1600) receiver_throttle = 2000;
  if (receiver_throttle < 1400) receiver_throttle = 1000;
  //Serial.println(receiver_throttle);
  set_motor_speeds_throttle();
  return;*/