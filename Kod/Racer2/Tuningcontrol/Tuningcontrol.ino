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


uint8_t buf[67];


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
  delay(2000);
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void get_data(int n){
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

   start_time = millis();
  
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

  
  Wire.write(buf, 67);
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


