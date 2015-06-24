#include <VirtualWire.h>

const int transmit_pin = 12;
const int receive_pin = 10;
const int transmit_en_pin = 11;

uint16_t joy_axis[4]; //right x,y left x,y
uint8_t rsw, last_rsw, lsw, last_lsw; //joystick switches
byte state, lights;

#define BUFFER_MAX VW_MAX_MESSAGE_LEN
byte buffer[BUFFER_MAX];
byte bufpos = 0;

#define K 0.059
#define M -30.0


void setup(){
  //init switches
  rsw = 0;
  last_rsw = 0;
  lsw = 0;
  last_lsw = 0;
  
  state = 0; //hover mode
  lights = 0; //off

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(6000); // Bits per sec
  
  //joystick inputs
  pinMode(A0,INPUT); //rx
  pinMode(A1,INPUT); //ry
  pinMode(A2,INPUT); //lx
  pinMode(A3,INPUT); //ly
  
  pinMode(2,INPUT); //lsw
  pinMode(3,INPUT); //rsw
  
  //led
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);


  
  Serial.begin(38400);
  Serial.println("setup");
}

void change_state(){
  ///hover/flight
  if (lsw != last_lsw)
  {
    if (lsw == 1)
    {
      if (state == 0)
      {
        state = 1;
      }
      else
      {
        state = 0;
      }
    }    
  }
  
  //lights
  if (rsw != last_rsw)
  {
    if (rsw == 1)
    {
      if (lights == 0)
      {
        lights = 1;
      }
      else
      {
        lights = 0;
      }
    }    
  }
}

void clear_buffer(){
  for (byte i = 0; i < BUFFER_MAX; i++)
  {
    buffer[i] = 0;
  }
  bufpos = 0;
}

void add_to_buffer(byte b){
  if (bufpos < BUFFER_MAX)
  {
    buffer[bufpos] = b;
    bufpos++;
  }  
}

void send_buffer(uint8_t * buf, uint8_t buflen){
  vw_send(buf,buflen);  
  vw_wait_tx(); //wait until the whole buffer has been sent
}

int get_desired_angle(uint16_t in){
  int t = 0 + in;
  double x = (double)t;

  return (int)(K*x + M);
  //return (t-509)/7;
}

void send_to_copter(){
  clear_buffer();
  
  //add states
  add_to_buffer(state);
  add_to_buffer(lights);
  
  //add desired tilt, roll and yaw angles
  
  add_to_buffer((byte)get_desired_angle(joy_axis[1])); //roll
  add_to_buffer((byte)get_desired_angle(joy_axis[0])); //pitch
  add_to_buffer((byte)get_desired_angle(joy_axis[3])); //yaw

  Serial.print(" roll = ");
  Serial.print(get_desired_angle(joy_axis[1]),DEC);
  
  Serial.print(", pitch = ");
  Serial.print(get_desired_angle(joy_axis[0]),DEC);
  
  Serial.print(", yaw = ");
  Serial.print(get_desired_angle(joy_axis[3]),DEC);
  
  Serial.print(", throttle = ");
  Serial.print(joy_axis[2],DEC);
  
  Serial.println();
  
  //add throttle
  add_to_buffer((uint8_t)((joy_axis[2] >> 2))); //pick most significant bits
  
  
  //send  
  send_buffer((uint8_t *)buffer, bufpos);
}


void loop(){
  //temporary delay for debug
  
  delay(50);
  
  digitalWrite(13,HIGH);
  
  //read joysticks
  joy_axis[0] = analogRead(A0);
  joy_axis[1] = analogRead(A1);
  joy_axis[2] = analogRead(A2);
  joy_axis[3] = analogRead(A3);
  
  last_lsw = lsw;
  lsw = digitalRead(2);  
  last_rsw = rsw;
  rsw = digitalRead(3);
  
  change_state();
  
  send_to_copter();

  
  digitalWrite(13,LOW);
  

}
