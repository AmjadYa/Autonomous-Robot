/**
 * This file has all the logic that the autonomous robot followed once placed down.
*/


#include "Arduino.h"
#include <Servo.h>
#include <limits.h>

//Speed limit
double divisor = 1.0;

//Reflectivity definitions
#define startSwitch PA4
#define frontEdgeDetection PA5

//IR definitions
#define rightLongIR PA2
#define rightShortIR PA3
#define leftLongIR PA0
#define leftShortIR PA1

//Sonar definitions
#define frontSonarOut PA11
#define frontSonarIn PA12
#define leftSonarOut PA15
#define leftSonarIn PB3
#define rightSonarOut PB4
#define rightSonarIn PB5

//Steering Definitions
#define leftMotor PB_7
#define backLeftMotor PB_6
#define backRightMotor PB_9
#define rightMotor PB_8
#define steerServo PA_9
#define bananaServo PA_8

//Max gains
#define maxi 3
#define maxg 50

//Zipline Definitions
#define ziplineServo PA_6

/* REUSED VARIABLES */

//Constants & Thresholds
const int IRThreshold = 510;
const double EdgeFollowThreshold = 470;
const int sideSonarThreshold = 18;
const int frontSonarThreshold = 16;
const int SampleSize = 60;
const int frontEdgeThreshold = 1000;
int delayTapeThreshold;
//const double corrThreshold = 5.5;
//const double corrThreshold = 5.15;
const double corrThreshold = 2.5;


//PID variables
double error;
double lastErr = 0;
double p;
double i = 0;
double d;
double g;
double T;

//Convolution variables
double leftConvolution = 0;
double rightConvolution = 0;
double leftShortReading;
double rightShortReading;
//Initialize an array with twice as many data points as the sampled wave.
double generatedOneK[SampleSize / 2];

//Sonar variables
double distanceFront = 1;
double distanceLeft = 1;
double distanceRight = 1;
double distance;
long durationFront;
long durationLeft;
long durationRight;
long duration;
int delayFrontReading;
bool firstEdge = false;
bool caughtRampEdge;

//Drive variables
int motorVal;
int steerVal = 1500;

//Starting stage
int stage = 1;
long delayTapeReading;
bool readHigh = false;
bool readLow = false;
bool isHit = false;
int boyVar;
double girlVal;

void hit();

/**
 * This function is a specifically hardcoded left turn maneuver.
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void goLeft(int delayVal)
{
  //Hardcoded sharp left turn
  pwm_start(leftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(steerServo, 50, 500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(backLeftMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  //Change this delay to vary the angle of the left turn
  delay(delayVal);
  pwm_start(backLeftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}//End goLeft

/**
 * This function goes slightly left
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void slightLeft()
{
  pwm_start(steerServo, 50, 1460, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, (int)(65535/(1.11*divisor)), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/**
 * This function makes the bot go straight
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
 * 
*/
void driveStraight()
{
  //hard code driving straight 
  pwm_start(steerServo, 50, 1500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/**
 * This function is what runs if we start on the right
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void startRight()
{
  driveStraight();
  delay(700);
  pwm_start(steerServo, 50, 2500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  delay(250);
  slightLeft();
}

/**
 * This function is what runs if we start on the left
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void startLeft()
{
  driveStraight();
  delay(700);
  /*pwm_start(steerServo, 50, 500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);*/
  goLeft(375);
  driveStraight();
}

/**
 * U-turns towards the ramp
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void uTurn()
{
  //hard code driving straight 
  pwm_start(steerServo, 50, 1000, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, (int)(65535/3*divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}//End uTurn

/**
 * This function stops the bot and straightens it out
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void stop()
{
  pwm_start(steerServo, 50, 1500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(leftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/**
 * Spins like a maniac
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void maniac()
{
  //Turn in place left
  pwm_start(steerServo, 50, 500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(backLeftMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(rightMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/**
 * This function is for backing up a slight amount after catching an edge
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void backUp(int delayVal)
{
  pwm_start(steerServo, 50, 1500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(backLeftMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(backRightMotor, 500, 65535, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  //Change this delay to change the amount the robot backs up
  delay(delayVal);
  pwm_start(backLeftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  pwm_start(backRightMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
}

/**
 * This function is for riding the zipline
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void zipline()
{
  //Change the value of the PWM pulse to make the servo clamp hard on the bar
  pwm_start(ziplineServo, 50, 1900, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  driveStraight();
  delay(1200);
  stop();
  //Change this delay value to release from the bar as easy as possible
  delay(2000);
  pwm_start(ziplineServo, 50, 650, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

}//End zipline

/**
 * This function lowers the claw so it can go under the bridge
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void lowerClaw()
{
  pwm_start(ziplineServo, 50, 650, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}//End lowerClaw

/**
 * Primes the claw to catch the zipline
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void primeClaw()
{
  pwm_start(ziplineServo, 50, 1200, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}//End primeClaw

/**
 * Drops a banana
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void dropBanana()
{
  pwm_start(bananaServo, 50, 1500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

/**
 * Primes the banana dropper
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void primeBanana()
{
  pwm_start(bananaServo, 50, 2300, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

/**
 * Bluepill setup function
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void setup()
{
  //Little green LED
  // pinMode(PC13, OUTPUT);

  //Sonar Declarations
  pinMode(leftSonarOut, OUTPUT);
  pinMode(leftSonarIn, INPUT_PULLUP);
  pinMode(rightSonarOut, OUTPUT);
  pinMode(rightSonarIn, INPUT_PULLUP);
  pinMode(frontSonarOut, OUTPUT);
  pinMode(frontSonarIn, INPUT_PULLUP);

  //IR Declarations
  pinMode(leftLongIR, INPUT);
  pinMode(rightLongIR, INPUT);
  pinMode(leftShortIR, INPUT);
  pinMode(rightShortIR, INPUT);

  //BABAOOEY
  pinMode(frontEdgeDetection, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(frontEdgeDetection), hit, RISING);

  primeBanana();

  //Fill in generatedOneK[] values with a classic 1khz wave at amplitude of 1
  for (int i = 0; i < SampleSize / 2; i++)
  {
    generatedOneK[i]=sin(i*TWO_PI/3);
  }

  //Start declaration
  pinMode(startSwitch, INPUT_PULLDOWN);

  if(digitalRead(startSwitch) == HIGH)
  {
    boyVar = 250;
    girlVal = 24;
    startLeft();
  }
  else
  {
    boyVar = 367;
    girlVal = 20;
    startRight();
  }

  //Don't use this if you're running the steer servo
  //Serial.begin(115200);

}//End Setup

/**
 * This is the motor function that controls the steering
 * 
 * Parameters: Gain comes from the PID and tells us how to steer.
 * 
 * Returns: Nothing
*/
void drive (double gain)
{
  pwm_start(backLeftMotor, 500, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);

  steerVal = (int)((gain/100) * 2000 + 500);
  pwm_start(steerServo, 50, steerVal, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //Go left at full speed
  if (gain > 50)
  {
    motorVal = (int)(65535 - (2*(gain-50)/100)*65535);

    pwm_start(leftMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(rightMotor, 500, motorVal/divisor, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  //Go right at full speed
  else if (gain < 50)
  {
    motorVal = (int)((2*(gain)/100)*65535);

    pwm_start(leftMotor, 500, motorVal/divisor, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  //Go straight
  else
  {
    pwm_start(leftMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(rightMotor, 500, (int)(65535/divisor), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  
}//End drive

/**
 * You need to convolute the IR since it is being pulsed at 1kHz.
 * We don't want our robot to accidentally follow any other IR signals.
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void convolution(int leftPin, int rightPin)
{
  //Sampling Stage
  double leftSamples[SampleSize];
  double rightSamples[SampleSize];
  double maxAmpLeft = 0;
  double maxAmpRight = 0;
  double maxOneLeft = -1000;
  double maxOneRight = -1000;

  long time1 = micros();
  int i = 0;

  while (i < SampleSize)
  {
    long time2 = micros();
    if(time2 - time1 > 333)
    {
      time1 = time2;

      leftSamples[i] = (double)analogRead(leftPin) - 512;
      maxAmpLeft = max(maxAmpLeft, abs(leftSamples[i]));
      rightSamples[i] = (double)analogRead(rightPin) - 512;
      maxAmpRight = max(maxAmpRight, abs(rightSamples[i]));

      i++;
    }//End if

  }//End reading loop

  //Normalize the samples
  for(int i = 0; i < SampleSize; i++)
  {
    leftSamples[i] = leftSamples[i]/maxAmpLeft;
    rightSamples[i] = rightSamples[i]/maxAmpRight;
  }

  //Initialize the (to-be) convoluted array
  double convolutionArrayLeft[SampleSize / 2];
  double convolutionArrayRight[SampleSize / 2];

  //Fill in convolutionArray[] values with a convolution
  for (int k = 0; k < SampleSize / 2; k++)
  {
    convolutionArrayLeft[k] = 0;
    convolutionArrayRight[k] = 0;

    for (int i = 0; i < SampleSize / 2; i++)
    {
      convolutionArrayLeft[k] += leftSamples[k + i] * generatedOneK[i];  
      convolutionArrayRight[k] += rightSamples[k + i] * generatedOneK[i]; 
    }
  }

  //Find the value that was most correlated (in phase)
  for (int i = 0; i < SampleSize / 2; i++)
  {
    if (convolutionArrayLeft[i] > maxOneLeft) maxOneLeft = convolutionArrayLeft[i];
    if (convolutionArrayRight[i] > maxOneRight) maxOneRight = convolutionArrayRight[i];
  }

  //Update convolution variables
  leftConvolution = (maxOneLeft > corrThreshold ? maxAmpLeft : 0);
  rightConvolution = (maxOneRight > corrThreshold ? maxAmpRight : 0);

}//End convolution

/**
 * This is the PID function that controls the way out robot responds to various
 * data. The goal is to control out steering servo and differential steering
 * in a way that allows it to smoothly navigate along the course.
 * 
 * Parameters: This function needs three gain coefficients which determine
 * it's reaction to various situations. It also needs two inputs (usually 
 * left and right inputs) that it can compare.
 * 
 * Returns: Nothing -> instead it calls the motor() function which controls
 * the PWM that drives the steering servo and the two back motors.
*/
void pid(int KP, int KI, int KD, double leftVal, double rightVal) 
{
  //Normalize left and right values between 0-1
  T = rightVal + leftVal;

  //If no idea where to go... go to last known location of correct direction
  if(T == 0)
  {
    error = lastErr;
    //stop();
    //maniac();
    //return;
  }
  else
  {
    rightVal = rightVal / T;
    leftVal = leftVal / T;
    //Error will be [-1,1]
    error = rightVal - leftVal;
  }

  //Proportional gain (how much should we ramp our steering to compensate for the error)
  p = KP*error;

  //Integral gain (what does the past tell us about our current steering position)
  i = KI*error+i;

  //Need if statements to keep integral gain under control
  if (i > maxi)
  {
    i = maxi;
  }
  else if (i < -maxi)
  {
    i = -maxi;
  }
  
  //Derivative gain (how smooth should we turn based on the last error)
  d = KD*(error-lastErr);

  //Sum all the gains and drive the motor
  g = p+i+d;

  //g should be [-50, 50]
  if (g > maxg)
  {
    g = maxg;
  }
  else if (g < -maxg)
  {
    g = -maxg;
  }

  lastErr = error;

  //Motor accepts values [0, 100]
  drive(g+50);

}//End pid

/**
 * This function performs sonar calcs
 * 
 * Paramters: int pinIn and pinOut -> the pins to be processed
 * 
 * Returns: distance
 * 
*/
double sonarFunction(int pinIn, int pinOut)
{
  //Clears the outputs
  digitalWrite(pinOut, LOW);
  delayMicroseconds(2);

  //Sets the outputs to HIGH for 10 micro seconds
  digitalWrite(pinOut, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinOut, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(pinIn, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  return distance;

}//End sonarFunction
/**
 * This is the main loop that controls the steering and block collection of the 
 * robot. Other functions may be implemented in in later stages given enough time.
 * 
 * Loop has five stages: (1) Tape following, (2) IR Following, (3) Transition to Edge following
 * (4) Edge Following and (5) Ziplining
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void loop()
{
  switch(stage)
  {
    //Stage 1 is catching the IR
    case 1:

      //Full speed start
      divisor = 1.0;

      //Set claw to lower height for bridge
      lowerClaw();
      
      //Moment you catch any 1k IR at all, head for it
      convolution(leftLongIR, rightLongIR);
      if (leftConvolution >= IRThreshold || rightConvolution >= IRThreshold)
      {
        stage++;
        break;
      }
      else
      {
        //IR PID
        pid(10,0,0,leftConvolution/500,rightConvolution/100);
      }

      break;

    //Stage 2 is long range IR following
    case 2:

      //Slow down ahead!
      divisor = 1.1;
      //Subtract  to account for physical offset difference

      convolution(leftShortIR, rightShortIR);

      //Test if you're close enough to the IR beacon
      if(isHit)
      {
        stage++;
        stop();
        delay(2000);

        break;
      }
      else
      {
        //IR PID
        pid(girlVal,0,0,leftConvolution/367,rightConvolution/boyVar);
      }

      break;

    //Stage 3 is transitioning to edge following
    case 3:

      //Start by slowing down
      backUp(180);
      stop();
      delay(300);
      //Turn left 90 degrees
      goLeft(640);
      stop();
      //delay700
      delay(300);
      //drive straight really slowly
      driveStraight();
      delay(230);
      divisor = 2.35;
      driveStraight();

      //Prime claw for catching the zipline
      primeClaw();

      stage++;
      break;

    //Stage 4 is edge following (ramp)
    case 4:
      
      distanceFront = sonarFunction(frontSonarIn, frontSonarOut);
      caughtRampEdge = false;

      //Wait until the front sonar catches the edge
      if (!firstEdge && distanceFront >= frontSonarThreshold)
      {
        //Back up and use the hard coded left turn again
        stop();
        backUp(300);
        stop();
        delay(400);
        goLeft(400);
        stop();
        delay(300);
        
        divisor = 1.25;
        firstEdge = true;
        delayFrontReading = millis();

      }//End transition to ramp

      //You are now on the ramp - start reading sonar data to stay on it
      else if (firstEdge)
      { 
        //Check if we're at the edge of the ramp, ready for the zipline
        if (distanceFront >= frontSonarThreshold && (millis() - delayFrontReading) > 1800)
        {
          stage++;
          firstEdge = false;
          break;
        }

        else if ((millis() - delayFrontReading) <= 1800)
        {
          distanceRight = sonarFunction(rightSonarIn, rightSonarOut);
          distanceLeft = sonarFunction(leftSonarIn, leftSonarOut);

          //If we're over the threshold on the right, go left
          if (distanceRight >= sideSonarThreshold)
          {
            //caughtRampEdge = true;
            pid(30,0,0,1,0);
          }
          //If we're over the threshold on the left, go right
          else if (distanceLeft >= sideSonarThreshold)
          {
            //caughtRampEdge = true;
            pid(10,0,0,0,1);
          }
          else //if (caughtRampEdge)
          {
            i = -2;
            pid(50,0,0,1,1);
          }
          
        }//End ramp following
        else
        {
          divisor = 2.3;
          driveStraight();
          dropBanana();
        }
          
      }//End zipline edge detection
        
      break;
  
    //Stage 5 is clamping and letting go of the zipline
    case 5:

      //Zipline time
      divisor = 1.9;
      stop();
      backUp(50);
      stop();
      divisor = 1.0;
      delay(700);
      zipline();
      delay(400);
      //Go left until you catch the IR again
      backUp(230);
      goLeft(775);
      divisor = 1.65;
      driveStraight();
      delay(1400);

      isHit = false;
      stage = 1;

      break;

  }//End switch

}//End Loop

/**
 * Interrupt to see if the robot hit the IR beacon yet. Updates a global variable.
 * 
 * Parameters: Nothing
 * 
 * Returns: Nothing
*/
void hit()
{
  isHit = true;
}

