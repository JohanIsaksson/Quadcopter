//Tuningcontrol.ino

#include "fixed_point.h"
#include <LiquidCrystal.h>

#define TOTAL_STATES 6
int state = 0;
bool in_state = false;

int btn_left_pin = 8, btn_middle_pin = 9, btn_right_pin = 10;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
double d = 5.001;

bool left, middle, right;
int left_last, middle_last, right_last;
uint32_t left_timer, middle_timer, right_timer;


void setup() {
  pinMode(btn_left_pin, INPUT_PULLUP);
  pinMode(btn_middle_pin, INPUT_PULLUP);
  pinMode(btn_right_pin, INPUT_PULLUP);
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print(d,10);
  Serial.begin(38400);
}



void loop() {

  check_buttons();
  //lcd.setCursor(0, 1);
  //lcd.print(1.0 / pow(2.0, TOTAL_BITS - INTEGER_BITS), 15);
  //lcd.print(decode(encode(d)),10);
  delay(100);

  if (left){
    digitalWrite(13, HIGH);
  }else{
    digitalWrite(13, LOW);
  }

  if (!in_state){    
    if (left){
      state--;
    }else if (middle){
      in_state = true;
      lcd.blink();
    }else if (right){
      state++;
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
      lcd.print(0.444, 10);
      lcd.setCursor(0, 1);
      if (in_state){
        
      }
      

      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print("I PITCH  -  ACRO");
      lcd.setCursor(0, 1);
      lcd.print(1, 10);
      lcd.setCursor(0, 0);

      break;

  case 2:
      lcd.setCursor(0, 0);
      lcd.print("D PITCH  -  ACRO");
      lcd.setCursor(0, 1);
      lcd.print(2, 10);
      lcd.setCursor(0, 0);

      break;

 case 3:
      lcd.setCursor(0, 0);
      lcd.print("P ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(3, 10);
      lcd.setCursor(0, 0);

      break;


  case 4:
      lcd.setCursor(0, 0);
      lcd.print("I ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(4, 10);
      lcd.setCursor(0, 0);

      break;

  case 5:
      lcd.setCursor(0, 0);
      lcd.print("D ROLL  -   ACRO");
      lcd.setCursor(0, 1);
      lcd.print(5, 10);
      lcd.setCursor(0, 0);

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
    if (millis() - left_timer > 1000){
      right = true;
    }    
    right_last = LOW;
  }else{
    right_last = HIGH;
    right = false;
  }
}


