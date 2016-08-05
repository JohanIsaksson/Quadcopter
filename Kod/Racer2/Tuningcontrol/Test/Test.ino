#include "fixed_point.h"
#include <Wire.h>

double P_pitch_a = 1.5, I_pitch_a = 0.1, D_pitch_a = 0.01,
        P_pitch_h = 1.0, I_pitch_h = 0.1, D_pitch_h = 0.01,
        P_roll_a = 1.5, I_roll_a = 0.1, D_roll_a = 0.01,
        P_roll_h = 1.0, I_roll_h = 0.1, D_roll_h = 0.01,
        P_yaw = 2.5, I_yaw = 0.1, D_yaw = 0.0;

uint16_t max_pitch = 180, max_roll = 180, max_yaw = 90;

bool exp_lin;

uint8_t buf[67];


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(millis() < 10000){
    send_data();
  }

  while(1){
    delay(1000);
    get_data();
  }
}

void send_data(){

  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];

  
  uint32_t data32;
  uint16_t data16;
  double buf_tmp[19] = {P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw};
  uint16_t buf_tmp2[4] = {max_pitch, max_roll, max_yaw, exp_lin};

  for (int i = 0; i < 15; i++){
    data32 = encode_d(buf_tmp[i]);
    for (int j = 3; j >=0; j--){
      buf[i*4+(3-j)] = (data32 >> (8*j)) & 0x000000FF;
    }
  }

  for (int i = 0; i < 4; i++){
    buf[60 + i*2] = buf_tmp2[i] >> 8;
    buf[60 + i*2 + 1] = buf_tmp[i];
  }

  buf[66] = exp_lin;

  Wire.beginTransmission(15);
  Wire.write(buf, 67);
  Wire.endTransmission();  

}


void get_data(){
  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    i++;
  }
  
  uint32_t data32;
  uint16_t data16;
  uint32_t buf_tmp[19];

  for (int i = 0; i < 15; i++){
    buf_tmp[i] = (buf[i*4] << 24) + (buf[i*4 + 1] << 16) + (buf[i*4 + 2] << 8) + buf[i*4 + 3];    
  }

  for (int i = 0; i < 4; i++){
    buf_tmp[15+i] = (buf[60 + i*2] << 8) + buf[60 + i*2 + 1];
  }

   exp_lin = buf[66];

   P_pitch_a = decode_d(buf_tmp[0]);
   I_pitch_a = decode_d(buf_tmp[1]);
   D_pitch_a = decode_d(buf_tmp[2]);
   
   P_pitch_h = decode_d(buf_tmp[3]);
   I_pitch_h = decode_d(buf_tmp[4]);
   D_pitch_h = decode_d(buf_tmp[5]);
   
   P_roll_a = decode_d(buf_tmp[6]);
   I_roll_a = decode_d(buf_tmp[7]);
   D_roll_a = decode_d(buf_tmp[8]);
   
   P_roll_h = decode_d(buf_tmp[9]);
   I_roll_h = decode_d(buf_tmp[10]);
   D_roll_h = decode_d(buf_tmp[11]);
   
   P_yaw = decode_d(buf_tmp[12]);
   I_yaw = decode_d(buf_tmp[13]);
   D_yaw = decode_d(buf_tmp[14]);
   
   max_pitch = buf_tmp[15];
   max_roll = buf_tmp[16];
   max_yaw = buf_tmp[17];

  
}

