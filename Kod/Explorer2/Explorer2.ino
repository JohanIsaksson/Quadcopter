
#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"

#define SPEED_MIN 1000
#define SPEED_MAX 2000

#define REF_MAX_HORIZON 25.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 180.0

#define INTEGRAL_MAX 150.0

#define PARAM_ADDR 0

#define LOST_CONNECTION_COUNT 100
#define LOST_CONNECTION_COUNT2 12000
#define EMERGENCY_THROTTLE 0

#define MODE_HORIZON 0
#define MODE_ACRO 1


//#define YPR_DATA
//#define COMP_DATA
//#define PID_DATA
//#define MAG_DATA
//#define JAVA_DATA
//#define RADIO_DATA
//#define ESC_DATA


// Tuning control
#define TUNING_MAX 10.0
#define TUNING_MIN 0.0
//#define TUNING_MODE 6
double tun;
/*
0   -   P pitch/roll acro
1   -   I pitch/roll acro
2   -   D pitch/roll acro
3   -   P yaw
4   -   I yaw
5   -   D yaw
6   -   P pitch/roll horizon
7   -   I pitch/roll horizon
8   -   D pitch/roll horizon
9   -   


*/

IMU imu;

// Motor signals
int throttle[4]; /* 0 = front left
                    1 = front right
                    2 = back left
                    3 = back right
                  */

// Motor pins
#define fl_pin 10 //front left
#define fr_pin 11 //front right
#define bl_pin 12 //back left
#define br_pin 13 //back right

// Faster IO for samd21
#ifdef _VARIANT_ARDUINO_ZERO_
  volatile uint32_t *set_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTCLR.reg;
  const uint32_t  fl_MASK = (1ul << g_APinDescription[fl_pin].ulPin);

  volatile uint32_t *set_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTCLR.reg;
  const uint32_t  fr_MASK = (1ul << g_APinDescription[fr_pin].ulPin);

  volatile uint32_t *set_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTCLR.reg;
  const uint32_t  bl_MASK = (1ul << g_APinDescription[bl_pin].ulPin);

  volatile uint32_t *set_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTCLR.reg;
  const uint32_t  br_MASK = (1ul << g_APinDescription[br_pin].ulPin);
#endif

// PID outputs for each axis
int front;
int left;
int cw;

// Horizon mode stabilzation outputs
int pitch_stab, roll_stab, yaw_stab;

// PID controllers for each axis
Pid pid_pitch_rate, pid_roll_rate, pid_yaw_rate;
Pid pid_pitch_stab, pid_roll_stab, pid_yaw_stab;

// PID parameters
double P_pitch_a, I_pitch_a, D_pitch_a,
        P_pitch_h, I_pitch_h, D_pitch_h,
        P_roll_a, I_roll_a, D_roll_a,
        P_roll_h, I_roll_h, D_roll_h,
        P_yaw, I_yaw, D_yaw;

// Maximum angle and rotation sppeds
uint16_t max_pitch, max_roll, max_yaw;
uint16_t max_p, max_r, max_y;

// L.O.S. safety counter
int radio_off_counter;

// Time keeping
uint32_t time_diff;
uint32_t time_last;
double timed;

// Radio variables
volatile byte last_channel_1;
volatile byte last_channel_2; 
volatile byte last_channel_3; 
volatile byte last_channel_4;
volatile byte last_channel_5; 
volatile byte last_channel_6;

volatile int receiver_input_channel_1;
volatile int receiver_input_channel_2; 
volatile int receiver_input_channel_3; 
volatile int receiver_input_channel_4; 
volatile int receiver_input_channel_5; 
volatile int receiver_input_channel_6;

volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
                  
double rad_roll, rad_pitch, rad_yaw;
uint16_t rad_throttle;

// Motor control variables
bool motors_on, disable_sticks;
uint32_t esc_time, start_time, end_time;

// Flight mode control
uint8_t flight_mode;

// Yaw control
bool set_yaw;

// Serial bus for SAMD21
#define SerialPort SerialUSB

// Mapping function for double data type
double map_d(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Initalizes ESCs by keeping throttle low for 3s. (MUST BE DONE ASAP AT POWER ON)
void init_motors(){
  uint32_t init_esc_time = micros() + 3000000;
  for(int i = 0; i < 4; i++){
    throttle[i] = SPEED_MIN;
  }

  while (micros() < init_esc_time){ //run for 3s approx

    start_time = micros();
    end_time = start_time + 2000;
    start_motor_pulses();

    esc_time = micros();

    while(end_time > esc_time){                         
      if (esc_time - start_time >= throttle[0]) *clr_fl = fl_MASK; //front left
      if (esc_time - start_time >= throttle[1]) *clr_fr = fr_MASK; //front right
      if (esc_time - start_time >= throttle[2]) *clr_bl = bl_MASK; //back left
      if (esc_time - start_time >= throttle[3]) *clr_br = br_MASK; //back right
      esc_time = micros();
    }

    stop_motor_pulses(); //for safety, set all outputs to 0*/


    /*PORTB |= B00001111; // set all inputs to 1

    esc_time = micros();

    while(end_time > esc_time){                         
      if (esc_time - start_time >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_time - start_time >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_time - start_time >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_time - start_time >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }

    PORTB &= B11110000; //for sfety, set all outputs to 0*/
      
    while(micros() - time_last < 4500);
  }
}

// Limit integer x to interval [a,b]
uint16_t limit(uint16_t x, uint16_t a, uint16_t b){
  uint16_t r;
  if (x < a) r = a;
  else if (x > b) r = b;
  return r;
}

// Sets all motor pulses high
void start_motor_pulses(){
  *set_fl = fl_MASK; //front left
  *set_fr = fr_MASK; //front right
  *set_bl = bl_MASK; //back left
  *set_br = br_MASK; //back right
}

// Sets all motor pulses low
void stop_motor_pulses(){
  *clr_fl = fl_MASK; //front left
  *clr_fr = fr_MASK; //front right
  *clr_bl = bl_MASK; //back left
  *clr_br = br_MASK; //back right
}

// Sets final throttle by combining outputs from all PIDs 
void set_motor_speeds(){
  //front left
  throttle[0] = rad_throttle + front + left + cw;

  //front right
  throttle[1] = rad_throttle + front - left - cw;

  //back left
  throttle[2] = rad_throttle - front + left - cw;

  //back right
  throttle[3] = rad_throttle - front - left + cw;

  //check and set speeds
  throttle[0] = limit(throttle[0], SPEED_MIN, SPEED_MAX);
  throttle[1] = limit(throttle[1], SPEED_MIN, SPEED_MAX);
  throttle[2] = limit(throttle[2], SPEED_MIN, SPEED_MAX);
  throttle[3] = limit(throttle[3], SPEED_MIN, SPEED_MAX);
    
  esc_time = micros();

  while(end_time > esc_time){
    if (esc_time - start_time >= throttle[0]) *clr_fl = fl_MASK; //front left
    if (esc_time - start_time >= throttle[1]) *clr_fr = fr_MASK; //front right
    if (esc_time - start_time >= throttle[2]) *clr_bl = bl_MASK; //back left
    if (esc_time - start_time >= throttle[3]) *clr_br = br_MASK; //back right
    esc_time = micros();
  }
  //turn all pulses off for safety
  stop_motor_pulses();

  if (flight_mode == MODE_HORIZON){
    while(micros() - time_last < 4000);
  }else if (flight_mode == MODE_ACRO){
    while(micros() - time_last < 2800);
  }
}

// Sets minimum throttle for all motors
void set_motor_speeds_min(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 1000;
  start_motor_pulses();


  while(end_time > micros());
  stop_motor_pulses();

    
  while(micros() - time_last < 4000);
}

void set_motor_speeds_max(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  start_motor_pulses();

  while(end_time > micros());
  stop_motor_pulses();

    
  while(micros() - time_last < 4000);
}


/*
  ####  ###### ###### ##  ## #####
 ##  ## ##       ##   ##  ## ##  ##
 ##     ##       ##   ##  ## ##  ##
  ####  ####     ##   ##  ## #####
     ## ##       ##   ##  ## ##
 ##  ## ##       ##   ##  ## ##
  ####  ######   ##    ####  ##
*/
void setup(){
  delay(2000);

  SerialPort.begin(115200);
  SerialPort.println("Starting up");

	//Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  //Mask:                       Channel:    Signal:
  /*PCMSK2 |= (1 << PCINT18); //  1           roll
  PCMSK2 |= (1 << PCINT19); //  2           pitch
  PCMSK2 |= (1 << PCINT20); //  3           throttle
  PCMSK2 |= (1 << PCINT21); //  4           yaw
  PCMSK2 |= (1 << PCINT22); //  5           ACRO/HORIZON
  PCMSK2 |= (1 << PCINT23); //  6           -*/

  radio_off_counter = 0;
  receiver_input_channel_1 = 1500;
  receiver_input_channel_2 = 1500;
  receiver_input_channel_3 = 1000;
  receiver_input_channel_4 = 1500;
  receiver_input_channel_5 = 1000;
  receiver_input_channel_6 = 1000;

  //retrieve pid parameters from eeprom
  #ifndef TUNING_MODE
    //EEPROM_get();
    /*SerialPort.print(P_pitch_a); SerialPort.print("\t"); SerialPort.print(I_pitch_a); SerialPort.print("\t"); SerialPort.println(D_pitch_a);
    SerialPort.print(P_yaw); SerialPort.print("\t"); SerialPort.print(I_yaw); SerialPort.print("\t"); SerialPort.println(D_yaw);
    SerialPort.print(P_pitch_h); SerialPort.print("\t"); SerialPort.print(I_pitch_h); SerialPort.print("\t"); SerialPort.println(D_pitch_h);
    SerialPort.println("Retrieved PID parameters.");*/
  #else
    /*EEPROM_get();
    SerialPort.print(P_pitch_a); SerialPort.print("\t"); SerialPort.print(I_pitch_a); SerialPort.print("\t"); SerialPort.println(D_pitch_a);
    SerialPort.print(1.2); SerialPort.print("\t"); SerialPort.print(0.8); SerialPort.print("\t"); SerialPort.println(0.06);


    SerialPort.println();
    SerialPort.print(P_pitch_h); SerialPort.print("\t"); SerialPort.print(I_pitch_h); SerialPort.print("\t"); SerialPort.println(D_pitch_h);
    SerialPort.print(4.0); SerialPort.print("\t"); SerialPort.print(0.0); SerialPort.print("\t"); SerialPort.println(0.0);

    SerialPort.println();
    SerialPort.print(P_yaw); SerialPort.print("\t"); SerialPort.print(I_yaw); SerialPort.print("\t"); SerialPort.println(D_yaw);
    SerialPort.print(2.0); SerialPort.print("\t"); SerialPort.print(0.0); SerialPort.print("\t"); SerialPort.println(0.0);*/


    P_pitch_a = 1.2;
    P_roll_a = 1.2;

    I_pitch_a = 0.8;
    I_roll_a = 0.8;

    D_pitch_a = 0.06;
    D_roll_a = 0.06;

    P_pitch_h = 4.0;
    P_roll_h = 4.0;

    P_yaw = 2.0;
    I_yaw = 0.8;

    //
    //EEPROM_put();
  #endif


  imu.Init();
	

  pid_pitch_rate.Init();
  pid_pitch_rate.SetConstants(P_pitch_a, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
  pid_roll_rate.Init();
  pid_roll_rate.SetConstants(P_roll_a, I_roll_a, D_roll_a, INTEGRAL_MAX);
  pid_yaw_rate.Init();
  pid_yaw_rate.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  pid_pitch_stab.Init();
  pid_pitch_stab.SetConstants(P_pitch_h, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
  pid_roll_stab.Init();
  pid_roll_stab.SetConstants(P_roll_h, I_roll_h, D_roll_h, INTEGRAL_MAX);
  pid_yaw_stab.Init();
  pid_yaw_stab.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  front = 0;
  left = 0;
  cw = 0;

  //wait for esc init;
  SerialPort.println("Initializing ESCs...");
  //asign motors to pins

  //set pins as outputs
  pinMode(fl_pin, OUTPUT);
  pinMode(fr_pin, OUTPUT);
  pinMode(bl_pin, OUTPUT);
  pinMode(br_pin, OUTPUT);

  //initialize escs
  init_motors();
  motors_on = false;
  //motors_on = true;


  time_last = micros();
  SerialPort.println("Running...");

  setup_radio();
}


/*
 ##  ##  ####  #####  #### ######  ####  ##  ##
 ##  ## ##  ## ##  ##  ##      ## ##  ## ### ##
 ##  ## ##  ## ##  ##  ##     ##  ##  ## ######
 ###### ##  ## #####   ##    ##   ##  ## ######
 ##  ## ##  ## ####    ##   ##    ##  ## ## ###
 ##  ## ##  ## ## ##   ##  ##     ##  ## ##  ##
 ##  ##  ####  ##  ## #### ######  ####  ##  ##
*/
void Update_horizon(uint32_t t){
  timed = (double)t/1000000.0;

  //Update all sensor on the imu
  imu.UpdateHorizon(timed); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  start_motor_pulses();


  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500; //roll
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500; //pitch
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515){                                  //yaw
    receiver_input_channel_4 = 1500;
    if (set_yaw){
      rad_yaw = imu.ypr[0];
    }
    set_yaw = false;
  }else{
    set_yaw = true;
  }

  //map inputs to angles
  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  //rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pid_pitch_stab.Update(&pitch_stab, rad_pitch, (imu.ypr[1]-0.4), timed, 1.0);  //(imu.ypr[1]-0.4)
  pid_roll_stab.Update(&roll_stab, rad_roll, (imu.ypr[2]-0.35), timed, 1.0);     //(imu.ypr[2]-0.3)
  pid_yaw_stab.Update(&yaw_stab, rad_yaw, (imu.ypr[0]), timed, 1.0);

  pid_pitch_rate.Update(&front, (double)pitch_stab, imu.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.Update(&left, (double)roll_stab, imu.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.Update(&cw, (double)yaw_stab, -imu.z_gyr*RAD_TO_DEG, timed, -1.0);

}

/*
   ##    ####  #####   ####  #####    ## ###### #### ####
  ####  ##  ## ##  ## ##  ## ##  ##  ####  ##    ## ##  ##
 ##  ## ##     ##  ## ##  ## ##  ## ##  ## ##    ## ##
 ###### ##     #####  ##  ## #####  ###### ##    ## ##
 ##  ## ##     ####   ##  ## ##  ## ##  ## ##    ## ##
 ##  ## ##  ## ## ##  ##  ## ##  ## ##  ## ##    ## ##  ##
 ##  ##  ####  ##  ##  ####  #####  ##  ## ##   #### ####
*/
void Update_acro(uint32_t t){
  timed = (double)t/1000000.0;

  //Update only the gyroscope on the imu
  imu.UpdateAcro(timed); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  start_motor_pulses();

  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1475 && receiver_input_channel_1 < 1525) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1475 && receiver_input_channel_2 < 1525) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1475 && receiver_input_channel_4 < 1525) receiver_input_channel_4 = 1500;

  //p.K_tmp = map_d((double)receiver_input_channel_6,1000.0, 2000.0, 0.0, 0.8);

  //map inputs to anglerates
  //rad_roll = map_x3((double)(receiver_input_channel_1-1500));
  //rad_pitch = map_x3((double)(receiver_input_channel_2-1500));
  //rad_yaw = map_x3((double)(receiver_input_channel_4-1500));

  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);
  
  //SerialPort.println(rad_roll); delay(50);


  //calculate pids
  pid_pitch_rate.Update(&front, rad_pitch, imu.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.Update(&left, rad_roll, imu.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.Update(&cw, rad_yaw, -imu.z_gyr*RAD_TO_DEG, timed, -1.0);
}


/*
 ##     ####   ####  #####
 ##    ##  ## ##  ## ##  ##
 ##    ##  ## ##  ## ##  ##
 ##    ##  ## ##  ## #####
 ##    ##  ## ##  ## ##
 ##    ##  ## ##  ## ##
 ###### ####   ####  ##
*/
void loop(){


  time_diff = micros() - time_last;
  time_last = micros();

  set_motor_speeds_min();

  //rad_throttle = map(receiver_input_channel_3, 1000, 2000, 1060, 1800);


  //SerialPort.println(imu.altitude);
  //SerialPort.print(", ");
  //SerialPort.print(imu.vertical_speed);
  //SerialPort.print(", ");
  //SerialPort.println(imu.vertical_acc);
  
  /*
  SerialPort.print(imu.x_acc);
  SerialPort.print(", ");
  SerialPort.println(imu.y_acc);
  */
  /*
  SerialPort.print(imu.ax);
  SerialPort.print(", ");
  SerialPort.print(imu.ay);
  SerialPort.print(", ");
  SerialPort.print(imu.az);
  SerialPort.print(", ");
  SerialPort.print(imu.gx);
  SerialPort.print(", ");
  SerialPort.print(imu.gy);
  SerialPort.print(", ");
  SerialPort.println(imu.gz);
  */
  /*
  SerialPort.print(imu.ypr[0]);
  SerialPort.print(", ");
  SerialPort.print(imu.ypr[1]);
  SerialPort.print(", ");
  SerialPort.println(imu.ypr[2]);
  */
  /*
  SerialPort.print(", ");
  SerialPort.print(imu.x_gyr);
  SerialPort.print(", ");
  SerialPort.print(imu.y_gyr);
  SerialPort.print(", ");
  SerialPort.println(imu.z_gyr);
  */

  delay(20);

  SerialPort.print(receiver_input_channel_1);
  SerialPort.print(", ");
  SerialPort.print(receiver_input_channel_2);
  SerialPort.print(", ");
  SerialPort.print(receiver_input_channel_3);
  SerialPort.print(", ");
  SerialPort.println(receiver_input_channel_4);

  return;

  //motor arming control
  /*if (receiver_input_channel_6 < 1300){
    motors_on = false;
  }else if (receiver_input_channel_6 > 1700){
    motors_on = true;
  }*/
  motors_on = true;

  //horizon stabilization or acrobatic mode
  if (receiver_input_channel_5 < 1300){
    flight_mode = MODE_HORIZON;    
  }else if(receiver_input_channel_5 > 1700){
    flight_mode = MODE_ACRO;
  }



  if (radio_off_counter >= LOST_CONNECTION_COUNT){
    receiver_input_channel_1 = 1500;
    receiver_input_channel_2 = 1500;
    receiver_input_channel_3 = 1100;
    receiver_input_channel_4 = 1500;
    receiver_input_channel_5 = 1000;

    if (radio_off_counter >= LOST_CONNECTION_COUNT2){
      receiver_input_channel_6 = 1000;
    }else{
      receiver_input_channel_6 = 2000;
    }
      
    

    //radio_off_counter++;



    #ifdef TUNING_MODE

      tun = map_d((double)receiver_input_channel_6, 1000.0, 2000.0, TUNING_MIN, TUNING_MAX);


      switch (TUNING_MODE) {
          case 0:            
            pid_pitch_rate.SetConstants(tun, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
            pid_roll_rate.SetConstants(tun, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
            break;
          case 1:
            pid_pitch_rate.SetConstants(P_pitch_a, tun, D_pitch_a, INTEGRAL_MAX);
            pid_roll_rate.SetConstants(P_pitch_a, tun, D_pitch_a, INTEGRAL_MAX);
            break;
          case 2:
            pid_pitch_rate.SetConstants(P_pitch_a, I_pitch_a, tun, INTEGRAL_MAX);
            pid_roll_rate.SetConstants(P_pitch_a, I_pitch_a, tun, INTEGRAL_MAX);
            break;
          case 3:
            pid_yaw_rate.SetConstants(tun, I_yaw, D_yaw, INTEGRAL_MAX);
            break;
          case 4:
            pid_yaw_rate.SetConstants(P_yaw, tun, D_yaw, INTEGRAL_MAX);
            break;
          case 5:
            pid_yaw_rate.SetConstants(P_yaw, I_yaw, tun, INTEGRAL_MAX);
            break;
          case 6:
            pid_pitch_stab.SetConstants(tun, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
            pid_roll_stab.SetConstants(tun, I_roll_h, D_roll_h, INTEGRAL_MAX);
            break;
          case 7:
            pid_pitch_stab.SetConstants(P_pitch_h, tun, D_pitch_h, INTEGRAL_MAX);
            pid_roll_stab.SetConstants(P_roll_h, tun, D_roll_h, INTEGRAL_MAX);
            break;
          case 8:
            pid_pitch_stab.SetConstants(P_pitch_h, I_pitch_h, tun, INTEGRAL_MAX);
            pid_roll_stab.SetConstants(P_roll_h, I_roll_h, tun, INTEGRAL_MAX);
            break;

      }

      SerialPort.println(tun); delay(500);

    #endif
    

    if (motors_on){
      //Update according to set flight mode
      if(flight_mode == MODE_HORIZON){
        Update_horizon(time_diff);
      }else if (flight_mode == MODE_ACRO){
        Update_acro(time_diff);
      }    
      //apply new speed to motors
      SerialPort.println(micros() - time_last);
      set_motor_speeds();
    }else{
      //keep motors Updated
      set_motor_speeds_min();
    }
  }
  //delay(10);
  //SerialPort.println(imu.ypr[1]);

  /*delay(25);
  SerialPort.print(receiver_input_channel_1);
  SerialPort.print("\t\t");

  SerialPort.print(receiver_input_channel_2);
  SerialPort.print("\t\t");

  SerialPort.print(receiver_input_channel_3);
  SerialPort.print("\t\t");

  SerialPort.print(receiver_input_channel_4);
  SerialPort.print("\t\t");

  SerialPort.print(receiver_input_channel_5);
  SerialPort.print("\t\t");

  SerialPort.print(receiver_input_channel_6);
  SerialPort.println("\t\t");*/
} 


/*
 #####    ##   ####  #### ####
 ##  ##  ####  ## ##  ## ##  ##
 ##  ## ##  ## ##  ## ## ##  ##
 #####  ###### ##  ## ## ##  ##
 ####   ##  ## ##  ## ## ##  ##
 ## ##  ##  ## ## ##  ## ##  ##
 ##  ## ##  ## ####  #### ####
*/

//int radio_roll_pin, radio_pitch_pin, radio_throttle_pin, radio_yaw_pin, radio_mode_pin, radio_kill_pin;
//assign radion pins
const byte radio_roll_pin = 9;
const byte radio_pitch_pin = 8;
const byte radio_throttle_pin = 7;
const byte radio_yaw_pin = 6;
const byte radio_mode_pin = 5;
const byte radio_kill_pin = 3;
  
void setup_radio(){

  pinMode(radio_roll_pin, INPUT);
  pinMode(radio_pitch_pin, INPUT);
  pinMode(radio_throttle_pin, INPUT);
  pinMode(radio_yaw_pin, INPUT);
  pinMode(radio_mode_pin, INPUT);
  pinMode(radio_kill_pin, INPUT);
   /*                                                                                       //  channel:
  attachInterrupt(digitalPinToInterrupt(radio_roll_pin), (voidFuncPtr)radio_roll_ISR, CHANGE);         //  1  
  attachInterrupt(digitalPinToInterrupt(radio_pitch_pin), (voidFuncPtr)radio_pitch_ISR, CHANGE);       //  2
  attachInterrupt(digitalPinToInterrupt(radio_throttle_pin), (voidFuncPtr)radio_throttle_ISR, CHANGE); //  3
  attachInterrupt(digitalPinToInterrupt(radio_yaw_pin), (voidFuncPtr)radio_yaw_ISR, CHANGE);           //  4
  attachInterrupt(digitalPinToInterrupt(radio_mode_pin), (voidFuncPtr)radio_mode_ISR, CHANGE);         //  5
  attachInterrupt(digitalPinToInterrupt(radio_kill_pin), (voidFuncPtr)radio_kill_ISR, CHANGE);         //  6*/

                                                                                           //  channel:
  attachInterrupt(digitalPinToInterrupt(radio_roll_pin), (voidFuncPtr)updateRadio, CHANGE);         //  1  
  attachInterrupt(digitalPinToInterrupt(radio_pitch_pin), (voidFuncPtr)updateRadio, CHANGE);       //  2
  attachInterrupt(digitalPinToInterrupt(radio_throttle_pin), (voidFuncPtr)updateRadio, CHANGE); //  3
  attachInterrupt(digitalPinToInterrupt(radio_yaw_pin), (voidFuncPtr)updateRadio, CHANGE);           //  4
  attachInterrupt(digitalPinToInterrupt(radio_mode_pin), (voidFuncPtr)updateRadio, CHANGE);         //  5
  attachInterrupt(digitalPinToInterrupt(radio_kill_pin), (voidFuncPtr)updateRadio, CHANGE);         //  6*/
  
}

void radio_roll_ISR(){
  updateRadio();
}

void radio_pitch_ISR(){
  updateRadio();
}

void radio_throttle_ISR(){
  updateRadio();  
}

void radio_yaw_ISR(){
  updateRadio();  
}

void radio_mode_ISR(){
  updateRadio();
}

void radio_kill_ISR(){
  updateRadio();
}

void updateRadio(){
  double us = micros();

  //Channel 1=========================================
  if(last_channel_1 == 0 && digitalRead(radio_roll_pin)){         //Input 9 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = us;                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(digitalRead(radio_roll_pin))){  //Input 9 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = us - timer_1;      //Channel 1 is micros() - timer_1
  }

  //Channel 2=========================================
  if(last_channel_2 == 0 && digitalRead(radio_pitch_pin)){         //Input 8 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = us;                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(digitalRead(radio_pitch_pin))){  //Input 8 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = us - timer_2;      //Channel 2 is micros() - timer_2
  }

  //Channel 3=========================================
  if(last_channel_3 == 0 && digitalRead(radio_throttle_pin)){         //Input 7 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = us;                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(digitalRead(radio_throttle_pin))){  //Input 7 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = us - timer_3;      //Channel 3 is micros() - timer_3
  }

  //Channel 4=========================================
  if(last_channel_4 == 0 && digitalRead(radio_yaw_pin)){         //Input 6 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = us;                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(digitalRead(radio_yaw_pin))){  //Input 6 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = us - timer_4;      //Channel 4 is micros() - timer_4
  }

    //Channel 5=========================================
  if(last_channel_5 == 0 && digitalRead(radio_mode_pin)){         //Input 5 changed from 0 to 1
    last_channel_5 = 1;                                 //Remember current input state
    timer_5 = us;                                 //Set timer_3 to micros()
  }
  else if(last_channel_5 == 1 && !(digitalRead(radio_mode_pin))){  //Input 5 changed from 1 to 0
    last_channel_5 = 0;                                 //Remember current input state
    receiver_input_channel_5 = us - timer_5;      //Channel 3 is micros() - timer_3
  }

  //Channel 6=========================================
  if(last_channel_6 == 0 && digitalRead(radio_kill_pin)){         //Input 3 changed from 0 to 1
    last_channel_6 = 1;                                 //Remember current input state
    timer_6 = us;                                 //Set timer_4 to micros()
  }
  else if(last_channel_6 == 1 && !(digitalRead(radio_kill_pin))){  //Input 3 changed from 1 to 0
    last_channel_6 = 0;                                 //Remember current input state
    receiver_input_channel_6 = us - timer_6;      //Channel 4 is micros() - timer_4
  }
}
