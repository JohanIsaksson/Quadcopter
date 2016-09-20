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
  Serial.begin(38400);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  send_data();
  delay(8000);
  

  while(1){
    delay(10000);
    get_data();
  }
}


void insert_32(uint32_t data, int pos){
  buf[pos] = (data >> 24) & 0x000000FF;
  buf[pos+1] = (data >> 16) & 0x000000FF;
  buf[pos+2] = (data >> 8) & 0x000000FF;
  buf[pos+3] = data & 0x000000FF;
}

void insert_16(uint16_t data, int pos){
  buf[pos] = (data >> 8) & 0x000000FF;
  buf[pos+1] = data & 0x000000FF;
}

void send_data(){

  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];


  //send first part
  buf[0] = 1;

  int pos = 1;

  insert_32(encode_d(P_pitch_a), pos);
  pos+=4;
  insert_32(encode_d(I_pitch_a), pos); 
  pos+=4;
  insert_32(encode_d(D_pitch_a), pos);
  pos+=4;
  insert_32(encode_d(P_pitch_h), pos); 
  pos+=4;
  insert_32(encode_d(I_pitch_h), pos); 
  pos+=4;
  insert_32(encode_d(D_pitch_h), pos); 

  Wire.beginTransmission(15);
  Wire.write(buf, 25);
  Wire.endTransmission(); 


  //send second part

  buf[0] = 2;

  pos = 1;

  insert_32(encode_d(P_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(I_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(D_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(P_roll_h), pos); 
  pos+=4;
  insert_32(encode_d(I_roll_h), pos); 
  pos+=4;
  insert_32(encode_d(D_roll_h), pos); 

  Wire.beginTransmission(15);
  Wire.write(buf, 25);
  Wire.endTransmission(); 


  // send third part

  buf[0] = 3;

  pos = 1;

  insert_32(encode_d(P_yaw), pos); 
  pos+=4;
  insert_32(encode_d(I_yaw), pos); 
  pos+=4;
  insert_32(encode_d(D_yaw), pos);
  pos+=4;

  insert_16(max_pitch, pos);
  pos+=2;
  insert_16(max_roll, pos);
  pos+=2;
  insert_16(max_yaw, pos);
  pos+=2;

  buf[19] = exp_lin;

  Wire.beginTransmission(15);
  Wire.write(buf, 20);
  Wire.endTransmission(); 
}


void get_data(){
  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];
  
  uint32_t buf_tmp[6];


  Wire.requestFrom(15, 25);

  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }



  Wire.requestFrom(15, 25);

  i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }


  Wire.requestFrom(15, 25);

  i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }
}

