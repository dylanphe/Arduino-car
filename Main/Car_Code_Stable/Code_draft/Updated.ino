#include <ECE3.h>

 //initialize pins
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_nslp_pin=11;  // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;

//declaring and initializing arrays of informations stored for calculation of error
uint16_t sensorValues[8]; // right -> left, 0 -> 7
int32_t minArray[8] = {598, 597, 639, 598, 551, 551, 668, 691};
int16_t maxArray[8] = {1902, 1903, 1861, 1902, 1182,  1931,  1832,  1809};
uint16_t norm[8];
int weight;
int normSum;

// Spd Constants
const int baseSpd = 120;
const int turnSpd = 85;
int finCount;
int pastDir;
const int timeToTurn = 700;

// PID Constants
const float kP = 0.11;
const float kD = 0.72;


///////////////////////////////////

void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  // Set up diode pins
  pinMode(LED_RF, OUTPUT);

  ECE3_Init();
  finCount = 0;
  pastDir = 0;

  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
}

void loop() 
{
  // put your main code here, to run repeatedly: 
  ECE3_read_IR(sensorValues);
  
  for (int i = 0; i < 8; i++)
  {
    if (sensorValues[i] > maxArray[i])
      norm[i] = 1000;
    else if (sensorValues[i] < minArray[i])
      norm[i] = 0;
    else 
      norm[i] = (sensorValues[i] - minArray[i]) * 1000/ maxArray[i];
  }

  normSum = norm[0]+norm[1]+norm[2]+norm[3]+norm[4]+norm[5]+norm[6]+norm[7];
  weight =(-8*norm[0]-4*norm[1]-2*norm[2]-norm[3]+norm[4]+2*norm[5]+4*norm[6]+8*norm[7])/4;
  float motorSpeed = kP*weight+kD*(weight-pastDir); 
  
  if (normSum >= 6000)
  {
    finCount++;
    switch(finCount)
    {
      //first line turning around
     case 1:
      digitalWrite(right_dir_pin, HIGH);
      analogWrite(right_pwm_pin, turnSpd);
      analogWrite(left_pwm_pin, turnSpd);
      delay(timeToTurn);
      digitalWrite(right_dir_pin, LOW);
      break;
      // Seeing the line at the end of the track and stopping
     case 2:
      analogWrite(right_pwm_pin, 0);
      analogWrite(left_pwm_pin, 0);
      digitalWrite(right_nslp_pin, LOW);
      digitalWrite(left_nslp_pin, LOW);
      break;
     default:
      break;
    }
  }
  else //Follow Track
  {
    // if the car is turning to the left a significant amount
        if(weight < -600)
        {
            analogWrite(right_pwm_pin, baseSpd + motorSpeed);
            analogWrite(left_pwm_pin, 0.6*(baseSpd - motorSpeed));
        }
        // if the car is turning to the right a signifcant amount
        else if(weight > 600)
        {
            analogWrite(right_pwm_pin, 0.6*(baseSpd + motorSpeed));
            analogWrite(left_pwm_pin, baseSpd - motorSpeed);
        }
        // if the car is not making any sharp turns
        else
        {
           analogWrite(right_pwm_pin, baseSpd + motorSpeed);
           analogWrite(left_pwm_pin, baseSpd - motorSpeed);
        }
        analogWrite(right_pwm_pin, baseSpd + motorSpeed);
        analogWrite(left_pwm_pin, baseSpd - motorSpeed);
  }
  delay(1);
  pastDir = weight;
}


//  ECE3_read_IR(sensorValues);
//
//  digitalWrite(LED_RF, HIGH);
//  delay(250);
//  digitalWrite(LED_RF, LOW);
//  delay(250);
//  for (unsigned char i = 0; i < 8; i++)
//  {
//    Serial.print(norm[i]);
//  Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//  }
//  Serial.println();
//
//  delay(1000);


      /*if(readingCount > 6000 && counter == 0) //Start of Path
    {
      analogWrite(left_pwm_pin,baseSpd);
      analogWrite(right_pwm_pin,baseSpd);
      delay(baseSpd*20);
      counter++;
    }
    else if(readingCount > 6000 && counter > 0)
      {
      while(1) //Turnaround
        {
        analogWrite(left_pwm_pin,turnSpd);
        analogWrite(right_pwm_pin,turnSpd);
        delay(turnSpd*1000);
        }
      }
      else //Follow Track
        {
        digitalWrite(left_dir_pin,LOW);
        leftSpd = baseSpd - weight*baseSpd/1000;
        rightSpd = baseSpd + weight*baseSpd/1000;
        analogWrite(left_pwm_pin,leftSpd);
        analogWrite(right_pwm_pin,rightSpd);
        }
        */
