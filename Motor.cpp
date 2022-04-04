#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int plus, int minus, int en_a, int en_b, int en_PWM) 
  {
    pinMode(42,OUTPUT);//enable shield
    pinMode(43,OUTPUT);
    pinMode(44,OUTPUT);
    pinMode(45,OUTPUT);
    digitalWrite(42,1);// 
    digitalWrite(43,1);// 
    digitalWrite(44,1);// 
    digitalWrite(45,1);// 

    pinMode(plus,OUTPUT);
    pinMode(minus,OUTPUT);
    pinMode(en_a,INPUT_PULLUP);
    pinMode(en_b,INPUT_PULLUP);
    pinMode(en_PWM, OUTPUT);
    Motor::plus = plus;
    Motor::minus = minus;
    Motor::en_a = en_a;
    Motor::en_b = en_b;
    Motor::en_PWM = en_PWM;
  }


void Motor::rotate(double value) 
  {
    if(value>=0)
      {
        int out = map(value, 0, 100, 0, 250);

        digitalWrite(plus,1);
        digitalWrite(minus,0);
        analogWrite(en_PWM,out);

      
      }
    else
      {
        
        int out = map(value, 0, -100, 0, 250);

        digitalWrite(plus,0);
        digitalWrite(minus,1);
        analogWrite(en_PWM,out);
        
      }
  }
