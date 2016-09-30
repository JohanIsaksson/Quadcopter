//Tuningcontrol.ino

#include "fixed_point.h"
#include <LiquidCrystal.h>
#include <Wire.h>

#define TOTAL_STATES 19
int state = 0;
bool in_state = false;

int btn_left_pin = 8, btn_middle_pin = 9, btn_right_pin = 10;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
double d = 5.001;

bool left, middle, right;
int left_last, middle_last, right_last;
uint32_t left_timer, middle_timer, right_timer;

double P_pitch_a, I_pitch_a, D_pitch_a,
        P_pitch_h, I_pitch_h, D_pitch_h,
        P_roll_a, I_roll_a, D_roll_a,
        P_roll_h, I_roll_h, D_roll_h,
        P_yaw, I_yaw, D_yaw;

uint16_t max_pitch, max_roll, max_yaw;

bool exp_lin;

bool data_catch;
uint32_t start_time;

int send_pos;

uint8_t buf[26];


void setup() {
  //setup button pins
  pinMode(btn_left_pin, INPUT_PULLUP);
  pinMode(btn_middle_pin, INPUT_PULLUP);
  pinMode(btn_right_pin, INPUT_PULLUP);
  
  lcd.begin(16, 2);
    
  Serial.begin(38400);


  //init as slave 15
  Wire.onReceive(get_data);
  Wire.onRequest(send_data);
  Wire.begin(15);

  //delay(2000);
  
  
}

void load_data(){
  //start up sequence
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("LOADING DATA... ");
  start_time = millis();
  while(millis() - start_time < 2000);
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("LOADING COMPLETE");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void get_data(int n){
  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("------------");

  uint32_t buf_tmp[6];
  
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

   start_time = millis();
  
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
  int pos = 0; 
  //send first part
  if (send_pos == 0){
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

    Wire.write(buf, 25);

  }
  else if (send_pos == 1){

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

    Wire.write(buf, 25);

  }
  else if (send_pos == 2){
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

    Wire.write(buf, 20);

  }


  send_pos = (send_pos + 1) % 3;
}



void loop() {
  if (millis() - start_time < 2000){
    load_data();
  }
  // update buttons
  check_buttons();
  
  delay(50);

  //
  if (!in_state){    
    if (left){
      state--;
      lcd.clear();
    }else if (middle){
      in_state = true;
      lcd.blink();
    }else if (right){
      state++;
      lcd.clear();
    }
    if (state < 0){
      state += TOTAL_STATES;
    }else if (state > TOTAL_STATES-1){
      state -= TOTAL_STATES;
    }
  }else{
    if (middle){
      in_state = false;
      lcd.noBlink();
    }
  }

  lcd.setCursor(0, 0);
  
  switch(state){
    case 0:      
      lcd.print("P PITCH  -  ACRO");
      lcd.setCursor(0, 1);
      lcd.print(P_pitch_a, 6);
      if (in_state){
        if (left){
          P_pitch_a -= 0.01;
        }else if (right){
          P_pitch_a += 0.01;
        }
      }
      

      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print("I PITCH  -  ACRO");
      lcd.setCursor(0, 1);
      lcd.print(I_pitch_a, 6);
      if (in_state){
        if (left){
          I_pitch_a -= 0.001;
        }else if (right){
          I_pitch_a += 0.001;
        }
      }
      break;

  case 2:
      lcd.setCursor(0, 0);
      lcd.print("D PITCH  -  ACRO");
      lcd.setCursor(0, 1);
      lcd.print(D_pitch_a, 6);
      if (in_state){
        if (left){
          D_pitch_a -= 0.0001;
        }else if (right){
          D_pitch_a += 0.0001;
        }
      }
      break;

 case 3:
      lcd.setCursor(0, 0);
      lcd.print("P ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(P_roll_a, 6);
      if (in_state){
        if (left){
          P_roll_a -= 0.01;
        }else if (right){
          P_roll_a += 0.01;
        }
      }
      break;


  case 4:
      lcd.setCursor(0, 0);
      lcd.print("I ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(I_roll_a, 6);
      if (in_state){
        if (left){
          I_roll_a -= 0.001;
        }else if (right){
          I_roll_a += 0.001;
        }
      }

      break;

  case 5:
      lcd.setCursor(0, 0);
      lcd.print("D ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(D_roll_a, 6);
      if (in_state){
        if (left){
          D_roll_a -= 0.0001;
        }else if (right){
          D_roll_a += 0.0001;
        }
      }

      break;
      
  case 6:      
      lcd.print("P PITCH  - HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(P_pitch_h, 6);
      if (in_state){
        if (left){
          P_pitch_h -= 0.01;
        }else if (right){
          P_pitch_h += 0.01;
        }
      }
      

      break;

    case 7:
      lcd.setCursor(0, 0);
      lcd.print("I PITCH  - HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(I_pitch_h, 6);
      if (in_state){
        if (left){
          I_pitch_h -= 0.001;
        }else if (right){
          I_pitch_h += 0.001;
        }
      }
      break;

  case 8:
      lcd.setCursor(0, 0);
      lcd.print("D PITCH  - HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(D_pitch_h, 6);
      if (in_state){
        if (left){
          D_pitch_h -= 0.0001;
        }else if (right){
          D_pitch_h += 0.0001;
        }
      }
      break;

 case 9:
      lcd.setCursor(0, 0);
      lcd.print("P ROLL  -  HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(P_roll_h, 6);
      if (in_state){
        if (left){
          P_roll_h -= 0.01;
        }else if (right){
          P_roll_h += 0.01;
        }
      }
      break;


  case 10:
      lcd.setCursor(0, 0);
      lcd.print("I ROLL  -  HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(I_roll_h, 6);
      if (in_state){
        if (left){
          I_roll_h -= 0.001;
        }else if (right){
          I_roll_h += 0.001;
        }
      }

      break;

  case 11:
      lcd.setCursor(0, 0);
      lcd.print("D ROLL  -  HORIZ");
      lcd.setCursor(0, 1);
      lcd.print(D_roll_h, 6);
      if (in_state){
        if (left){
          D_roll_h -= 0.0001;
        }else if (right){
          D_roll_h += 0.0001;
        }
      }

      break;

  case 12:
      lcd.setCursor(0, 0);
      lcd.print("P YAW           ");
      lcd.setCursor(0, 1);
      lcd.print(P_yaw,6);
      if (in_state){
        if (left){
          P_yaw -= 0.01;
        }else if (right){
          P_yaw += 0.01;
        }
      }
      break;


  case 13:
      lcd.setCursor(0, 0);
      lcd.print("I YAW           ");
      lcd.setCursor(0, 1);
      lcd.print(I_yaw,6);
      if (in_state){
        if (left){
          I_yaw -= 0.001;
        }else if (right){
          I_yaw += 0.001;
        }
      }

      break;

  case 14:
      lcd.setCursor(0, 0);
      lcd.print("D YAW           ");
      lcd.setCursor(0, 1);
      lcd.print(D_yaw, 6);
      if (in_state){
        if (left){
          D_yaw -= 0.0001;
        }else if (right){
          D_yaw += 0.0001;
        }
      }

      break;

  case 15:
      lcd.setCursor(0, 0);
      lcd.print("MAX PITCH ROT.  ");
      lcd.setCursor(11,1);
      lcd.print("DEG/S");
      lcd.setCursor(0, 1);
      lcd.print(max_pitch);
      if (in_state){
        if (left){
          max_pitch -= 1;
        }else if (right){
          max_pitch += 1;
        }
      }
      break;


  case 16:
      lcd.setCursor(0, 0);
      lcd.print("MAX ROLL ROT.   ");
      lcd.setCursor(11,1);
      lcd.print("DEG/S");
      lcd.setCursor(0, 1);
      lcd.print(max_roll);
      if (in_state){
        if (left){
          max_roll -= 1;
        }else if (right){
          max_roll += 1;
        }
      }

      break;

  case 17:
      lcd.setCursor(0, 0);
      lcd.print("MAX YAW ROT.    ");
      lcd.setCursor(11,1);
      lcd.print("DEG/S");
      lcd.setCursor(0, 1);
      lcd.print(max_yaw);
      if (in_state){
        if (left){
          max_yaw -= 1;
        }else if (right){
          max_yaw += 1;
        }
      }

      break;

  case 18:
      lcd.setCursor(0, 0);
      lcd.print("JOYSTICK CONFIG.");
      lcd.setCursor(0, 1);
      lcd.print(exp_lin ? "EXPONENTIAL     " : "LINEAR          ");
      if (in_state){
        if (left){
          exp_lin = ! exp_lin;
        }else if (right){
          exp_lin = ! exp_lin;
        }
      }

      break;
    
  }
}





void check_buttons(){
  
  if (digitalRead(btn_left_pin) == LOW){     
    if (left_last == HIGH){ //pressed
      left = true;
      left_timer = millis();
    }else{
      left = false;
    }
    if (millis() - left_timer > 1000){
      left = true;
    }    
    left_last = LOW;
  }else{
    left_last = HIGH;
    left = false;
  }

  if (digitalRead(btn_middle_pin) == LOW){ 
    if (middle_last == HIGH){ //pressed
      middle = true;
      middle_timer = millis();
    }else{
      middle = false;
    }
    if (millis() - middle_timer > 1000){
      middle = true;
    }    
    middle_last = LOW;
  }else{
    middle_last = HIGH;
    middle = false;
  }


  if (digitalRead(btn_right_pin) == LOW){ 
    if (right_last == HIGH){ //pressed
      right = true;
      right_timer = millis();
    }else{
      right = false;
    }
    if (millis() - right_timer > 1000){
      right = true;
    }    
    right_last = LOW;
  }else{
    right_last = HIGH;
    right = false;
  }
}


