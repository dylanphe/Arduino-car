#include <ECE3.h>

//Initialize Pins
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29; // controls direction of motor: LOW -> Forward , HIGH -> Reversed
const int left_pwm_pin=40; // controls pwm of motor, set using: analogWrite(Pin_Name, Value)

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

// declaring and initializing arrays of informations stored for calculation of error
uint16_t sensorValues[8]; // right -> left, 0 -> 7
int32_t minArray[8] = {598, 597, 639, 598, 551, 551, 668, 691};
int16_t maxArray[8] = {1902, 1903, 1861, 1902, 1182,  1931,  1832,  1809};
uint16_t norm[8];
int normSum;
int finCount; // Number of times the car has seen the black turnaround/finish line
int errorNew; // Error of the current instance
int errorPrev; // Error of the previous instance, used to calculate the change in error
float addSpd; // Additional speed to make the car turn

// Spd and Delay Constants
const int baseSpd = 130;
const int turnSpd = 150; // Turning speed at the turnaround line
const int turnTime = 400; // Length, in time, of the turn
const float turnShrp = 0.65; // Constant used to slow a wheel for sharper turns

// PID Constants
const float kP = 0.10; 
const float kD = 0.7;

void setup() 
{
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  ECE3_Init();
  finCount = 0;
  errorPrev = 0;

  delay(1000); // Start after 1 second
}

void loop() 
{  
  ECE3_read_IR(sensorValues);
  for (int i = 0; i < 8; i++)
  {
    if (sensorValues[i] - minArray[i] > maxArray[i]) // norm[i] <= 1000
    {
      norm[i] = 1000;
    }
    else if (sensorValues[i] < minArray[i]) // norm[i] >= 0
    {
      norm[i] = 0;
    }
    else 
    {
      norm[i] = (sensorValues[i] - minArray[i]) * 1000/ maxArray[i];
    }  
  }
  
  normSum = norm[0] + norm[1] + norm[2] + norm[3] + norm[4] + norm[5] + norm[6] + norm[7];
  errorNew =(-8*norm[0] - 4*norm[1] - 2*norm[2] - norm[3] + norm[4] + 2*norm[5] + 4*norm[6] + 8*norm[7])/4;
  addSpd = kP*errorNew + kD*(errorNew-errorPrev); 
  
  if (normSum >= 6000)
  {
    finCount++;
    switch(finCount)
    {
     case 1: // Turnaround Line
       digitalWrite(right_dir_pin, HIGH);
       analogWrite(right_pwm_pin, turnSpd);
       analogWrite(left_pwm_pin, turnSpd);
       delay(turnTime);
       break;
     case 2: // Finish Line
        analogWrite(right_pwm_pin, 0);
        analogWrite(left_pwm_pin, 0);
        digitalWrite(right_nslp_pin, LOW);
        digitalWrite(left_nslp_pin, LOW);
        break;
     default:
        break;
    }
  }
  else // Follow Track
  {    
   digitalWrite(right_dir_pin, LOW);        
   if(addSpd < -30) // Sharp Left
      {
        analogWrite(right_pwm_pin, baseSpd + addSpd);
        analogWrite(left_pwm_pin, turnShrp*(baseSpd - addSpd));
      }
    else if(addSpd > 30) // Sharp Right
      {
        analogWrite(right_pwm_pin, turnShrp*(baseSpd + addSpd));
        analogWrite(left_pwm_pin, baseSpd - addSpd);
      }
    else // No Sharp Turns
      {
        analogWrite(right_pwm_pin, baseSpd + addSpd);
        analogWrite(left_pwm_pin, baseSpd - addSpd);
      }
  }
  errorPrev = errorNew;
  delay(1);
}
