/*  Artificial Neural Network Motor Control */
/* 
 *  This program trains an ANN to control the two individual motors of the robot. 
 *  The training set is shown below.
 *  Once trained it will drive forward and attempt to avoid obstacles. 
 */

 /* For SA_Bot V0.02 */
/* 
 *  Sean Hodgins 02/12/2016
 *  Creative Commons V3.0 Share-alike 
*/


#include <math.h>
#define DEBUG

#define LEDYEL 25
#define LEDRED 26

///////SHARP IR DISTANCE SENSORS
#define IR1 A2
#define IR2 A3
#define IR3 A1

////////MOTORS ENCODERS AND CONTROL
#define AENC1 A4
#define AENC2 A5
#define BENC1 13
#define BENC2 12

#define AIN1 5
#define AIN2 44
#define BIN1 8
#define BIN2 9

///////SONAR PING SENSORS
#define TRIG1 21
#define ECHO1 20
#define TRIG2 7
#define ECHO2 6
#define TRIG3 4
#define ECHO3 3

////////BUTTONS
#define BUT_RED 11
#define BUT_BLUE 10

/////ENCODER VARS
volatile int lastEncodedA = 0;
volatile long encoderValueA = 0;
long lastencoderValueA = 0;

volatile int lastEncodedB = 0;
volatile long encoderValueB = 0;
long lastencoderValueB = 0;


/******************************************************************
   Network Configuration - customized per network
 ******************************************************************/

const int PatternCount = 7;
const int InputNodes = 3;
const int HiddenNodes = 4;
const int OutputNodes = 2;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

float Input[PatternCount][InputNodes] = {
  { 0, 1, 1 },  // STRAIGHT AND RIGHT IS OPEN
  { 0, 1, 0 },  // STRAIGHT IS OPEN
  { 1, 1, 1 },  // LEFT, RIGHT, and STRAIGHT IS OPEN
  { 1, 1, 0 },  // STRAIGHT AND LEFT IS OPEN
  { 0, 0, 1 },  // RIGHT IS OPEN
  { 1, 0, 0 },  // LEFT IS OPEN
  { 0, 0, 0 },  // CORNERED

};

const float Target[PatternCount][OutputNodes] = {
  { 1, 0.5 },   //MOTOR LEFT FULL FORWARD, RIGHT STOPPED
  { 1, 1 },     //BOTH MOTORS FULL FORWARD
  { 1, 1 },     //BOTH MOTORS FULL FORWARD
  { 0.5, 1 },   //MOTOR RIGHT FULL FORWARD, LEFT STOPPED
  { 0.5, 0.2 }, //MOTOR RIGHT BACKWARDS, LEFT STOPPED
  { 0.2, 0.5 }, //MOTOR LEFT BACKWARDS, RIGHT STOPPED
  { 0.3, 0.3}   //BOTH MOTORS BACKWARDS
};

/******************************************************************
   End Network Configuration
 ******************************************************************/


int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long  TrainingCycle;
float Rando;
float Error = 2;
float Accum;


float Hidden[HiddenNodes];
float Output[OutputNodes];


float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

/********Button States***********/

int BUT_REDstate = 0;
int BUT_BLUEstate = 0;


/****MOTOR SETTINGS***/

int minABackward = 0;
int minAForward = 0;
int minBBackward = 0;
int minBForward = 0;



void setup() {
  pinMode(TRIG1, OUTPUT);           //Enable the sonar Sensor pins.
  pinMode(TRIG2, OUTPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);            //

  pinMode(IR1, INPUT);              //Configure IR Pins (ADC)
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);              //
  delay(1000);

  pinMode(BUT_RED, INPUT_PULLUP);   //Enable Red and Blue Buttons and set internal PULLUP resistor
  pinMode(BUT_BLUE, INPUT_PULLUP);

  pinMode(AIN1, INPUT);             //Configure Motor Pins
  pinMode(AIN2, INPUT);
  pinMode(BIN1, INPUT);
  pinMode(BIN2, INPUT);


  SerialUSB.begin(115200);          //Enable Serial for USB(For debugging)
  //while (!SerialUSB);


  analogWriteResolution(10);        //Set the analog resolution to 10-bit (1024) instead of default 8-bit.
  randomSeed(analogRead(A1));       //Collect a random ADC sample for Randomization.
  ReportEvery1000 = 1;
  for ( p = 0 ; p < PatternCount ; p++ ) {
    RandomizedIndex[p] = p ;
  }


  int ledYELState = LOW;
  unsigned long previousMillis = 0;
  const long interval = 1000;

  BUT_REDstate = digitalRead(BUT_RED);
  BUT_BLUEstate = digitalRead(BUT_BLUE);
  while (Serial1.available() == 0 && SerialUSB.available() == 0 && BUT_REDstate == HIGH) {
    BUT_REDstate = digitalRead(BUT_RED); //Wait for input from either Serial
    BUT_BLUEstate = digitalRead(BUT_BLUE);

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledYELState == LOW) {
        ledYELState = HIGH;
      } else {
        ledYELState = LOW;
      }
      digitalWrite(LEDYEL, ledYELState);
    }
  }
  digitalWrite(LEDYEL, LOW);
  Serial1.read();
  while (BUT_REDstate == LOW) {           ///If RED button pushed, train neural network.
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
    if (BUT_REDstate == HIGH) {
      SerialUSB.println("Training New ANN");
    }
  }
  while (BUT_BLUEstate == LOW) {          //// IF BLUE button pushed, use pre-trained weights.
    BUT_BLUEstate = digitalRead(BUT_BLUE);
    delay(50);
    if (BUT_BLUEstate == HIGH) {
      SerialUSB.println("Using Previous ANN Settings");
    }
    Error = 0.00035;
  }

  CalibrateMinMotor();  ///Calibrates the minimum motor voltage
}

void loop () {

  // MotorTest();  //For Debugging
  //while (1);

  /** This loop runs when ANN has completed training **/
  while (Error < Success) { 
    int num;
    int farDist = 35;
    int closeDist = 7;
    float TestInput[] = {0, 0, 0};
    digitalWrite(LEDYEL, LOW);

    int ping1 = PING(ECHO1, TRIG1);   // Collect sonar distances.
    delay(10);
    int ping2 = PING(ECHO2, TRIG2);
    delay(10);
    int ping3 = PING(ECHO3, TRIG3);
#ifdef DEBUG
    SerialUSB.print("Ping: ");
    SerialUSB.print(ping1);
    SerialUSB.print("\t");
    SerialUSB.print(ping2);
    SerialUSB.print("\t");
    SerialUSB.println(ping3);
#endif

    digitalWrite(LEDYEL, HIGH);

    ping1 = map(ping1, closeDist, farDist, 0, 100);
    ping2 = map(ping2, closeDist, farDist, 0, 100);
    ping3 = map(ping3, closeDist, farDist, 0, 100);

    ping1 = constrain(ping1, 0, 100);
    ping2 = constrain(ping2, 0, 100);
    ping3 = constrain(ping3, 0, 100);

    TestInput[0] = float(ping1) / 100;
    TestInput[1] = float(ping2) / 100;
    TestInput[2] = float(ping3) / 100;
#ifdef DEBUG
    SerialUSB.print("Input: ");
    SerialUSB.print(TestInput[2], 2);
    SerialUSB.print("\t");
    SerialUSB.print(TestInput[1], 2);
    SerialUSB.print("\t");
    SerialUSB.println(TestInput[0], 2);
#endif

    InputToOutput(TestInput[2], TestInput[1], TestInput[0]);

    int speedA = Output[0] * 100;
    int speedB = Output[1] * 100;
    speedA = int(speedA);
    speedB = int(speedB);
#ifdef DEBUG
    SerialUSB.print("Speed: ");
    SerialUSB.print(speedA);
    SerialUSB.print("\t");
    SerialUSB.println(speedB);
#endif
    motorA(speedA);
    motorB(speedB);
    delay(50);
  }
  /******************************************************************
    Initialize HiddenWeights and ChangeHiddenWeights
  ******************************************************************/
  digitalWrite(LEDYEL, LOW);
  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    for ( j = 0 ; j <= InputNodes ; j++ ) {
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  digitalWrite(LEDYEL, HIGH);
  /******************************************************************
    Initialize OutputWeights and ChangeOutputWeights
  ******************************************************************/
  digitalWrite(LEDRED, LOW);
  for ( i = 0 ; i < OutputNodes ; i ++ ) {
    for ( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  digitalWrite(LEDRED, HIGH);
  //SerialUSB.println("Initial/Untrained Outputs: ");
  //toTerminal();
  /******************************************************************
    Begin training
  ******************************************************************/

  for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

    /******************************************************************
      Randomize order of training patterns
    ******************************************************************/

    for ( p = 0 ; p < PatternCount ; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p] ;
      RandomizedIndex[p] = RandomizedIndex[q] ;
      RandomizedIndex[q] = r ;
    }
    Error = 0.0 ;
    /******************************************************************
      Cycle through each training pattern in the randomized order
    ******************************************************************/
    for ( q = 0 ; q < PatternCount ; q++ ) {
      p = RandomizedIndex[q];

      /******************************************************************
        Compute hidden layer activations
      ******************************************************************/
      digitalWrite(LEDYEL, LOW);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = HiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[p][j] * HiddenWeights[j][i] ;
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
      }
      digitalWrite(LEDYEL, HIGH);

      /******************************************************************
        Compute output layer activations and calculate errors
      ******************************************************************/
      digitalWrite(LEDRED, LOW);
      for ( i = 0 ; i < OutputNodes ; i++ ) {
        Accum = OutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
      }
      //SerialUSB.println(Output[0]*100);
      digitalWrite(LEDRED, HIGH);
      /******************************************************************
        Backpropagate errors to hidden layer
      ******************************************************************/
      digitalWrite(LEDYEL, LOW);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = 0.0 ;
        for ( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }
      digitalWrite(LEDYEL, HIGH);

      /******************************************************************
        Update Inner-->Hidden Weights
      ******************************************************************/

      digitalWrite(LEDRED, LOW);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }
      digitalWrite(LEDRED, HIGH);
      /******************************************************************
        Update Hidden-->Output Weights
      ******************************************************************/
      digitalWrite(LEDYEL, LOW);
      for ( i = 0 ; i < OutputNodes ; i ++ ) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }
      digitalWrite(LEDYEL, HIGH);
    }

    /******************************************************************
      Every 1000 cycles send data to terminal for display
    ******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0)
    {
      SerialUSB.println();
      SerialUSB.println();
      SerialUSB.print ("TrainingCycle: ");
      SerialUSB.print (TrainingCycle);
      SerialUSB.print ("  Error = ");
      SerialUSB.println (Error, 5);

      toTerminal();

      if (TrainingCycle == 1)
      {
        ReportEvery1000 = 999;
      }
      else
      {
        ReportEvery1000 = 1000;
      }
    }


    /******************************************************************
      If error rate is less than pre-determined threshold then end
    ******************************************************************/

    if ( Error < Success ) break ;
  }
  SerialUSB.println ();
  SerialUSB.println();
  SerialUSB.print ("TrainingCycle: ");
  SerialUSB.print (TrainingCycle);
  SerialUSB.print ("  Error = ");
  SerialUSB.println (Error, 5);



  toTerminal();

  SerialUSB.println ();
  SerialUSB.println ();
  SerialUSB.println ("Training Set Solved! ");
  SerialUSB.println ("--------");
  SerialUSB.println ();
  SerialUSB.println ();
  ReportEvery1000 = 1;
  SerialUSB.println("Testing the output: ");


  SerialUSB.println("HIDDEN Weights");
  for (int i = 0 ; i < HiddenNodes ; i++ ) {
    for (int j = 0 ; j < InputNodes ; j++ ) {
      SerialUSB.print(j);
      SerialUSB.print(",");
      SerialUSB.print(i);
      SerialUSB.print(": ");
      SerialUSB.println(HiddenWeights[j][i]);
    }
  }
  SerialUSB.println("OUTPUT Weights");
  for (int i = 0 ; i < OutputNodes ; i++ ) {
    for (int j = 0 ; j < HiddenNodes ; j++ ) {
      SerialUSB.print(j);
      SerialUSB.print(",");
      SerialUSB.print(i);
      SerialUSB.print(": ");
      SerialUSB.println(OutputWeights[j][i]);
    }
  }

  int ledYELState = LOW;
  unsigned long previousMillis = 0;
  const long interval = 1000;
  SerialUSB.println("Press Red to re-train, Blue to Navigate");
  BUT_REDstate = digitalRead(BUT_RED);
  BUT_BLUEstate = digitalRead(BUT_BLUE);
  while (BUT_REDstate == HIGH && BUT_BLUEstate == HIGH) {
    BUT_REDstate = digitalRead(BUT_RED);
    BUT_BLUEstate = digitalRead(BUT_BLUE);

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledYELState == LOW) {
        ledYELState = HIGH;
      } else {
        ledYELState = LOW;
      }
      digitalWrite(LEDYEL, ledYELState);
    }
  }
  digitalWrite(LEDYEL, LOW);
  Serial1.read();
  while (BUT_REDstate == LOW) {           ///If RED button pushed, train neural network.
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
    if (BUT_REDstate == HIGH) {
      SerialUSB.println("Training New ANN");
      Error = 1;
    }
  }
  while (BUT_BLUEstate == LOW) {          //// IF BLUE button pushed, use pre-trained weights.
    BUT_BLUEstate = digitalRead(BUT_BLUE);
    delay(50);
    if (BUT_BLUEstate == HIGH) {
      SerialUSB.println("Driving");
    }
  }
  digitalWrite(LEDYEL, HIGH);
  delay(500);

  digitalWrite(LEDYEL, LOW);
  delay(500);
  digitalWrite(LEDYEL, HIGH);
  delay(500);

  digitalWrite(LEDYEL, LOW);
  delay(500);


}

void toTerminal()
{

  for ( p = 0 ; p < PatternCount ; p++ ) {
    SerialUSB.println();
    SerialUSB.print ("  Training Pattern: ");
    SerialUSB.println (p);
    SerialUSB.print ("  Input ");
    for ( i = 0 ; i < InputNodes ; i++ ) {
      SerialUSB.print (Input[p][i], DEC);
      SerialUSB.print (" ");
    }
    SerialUSB.print ("  Target ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      SerialUSB.print (Target[p][i], DEC);
      SerialUSB.print (" ");
    }
    /******************************************************************
      Compute hidden layer activations
    ******************************************************************/

    for ( i = 0 ; i < HiddenNodes ; i++ ) {
      Accum = HiddenWeights[InputNodes][i] ;
      for ( j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[p][j] * HiddenWeights[j][i] ;


      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }

    /******************************************************************
      Compute output layer activations and calculate errors
    ******************************************************************/

    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Accum = OutputWeights[HiddenNodes][i] ;
      for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }
    //SerialUSB.print ("  Output ");
    //for ( i = 0 ; i < OutputNodes ; i++ ) {
    //  SerialUSB.print (Output[i], 5);
    //  SerialUSB.print (" ");
    // }
  }


}

/*
 * This takes the Input from the sensors, and returns the Outputs of the motors.
 */
void InputToOutput(float In1, float In2, float In3) 
{
  float TestInput[] = {0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    Accum = HiddenWeights[InputNodes][i] ;
    for ( j = 0 ; j < InputNodes ; j++ ) {
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = OutputWeights[HiddenNodes][i] ;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
      Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }
#ifdef DEBUG
  SerialUSB.print ("  Output ");
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    SerialUSB.print (Output[i], 5);
    SerialUSB.print (" ");
  }
#endif
}


  /*
   * Left Motor Control Procedure - 0-49 - Reverse, 51-100 - Forward
   */
void motorA(int percent) {
  int maxSpeed = 85;
  int minSpeed = 45;
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {
    //SerialUSB.print("Driving ");
    //SerialUSB.println(percent);
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    digitalWrite(AIN1, LOW);
    int drive = map(percent, 51, 100, minAForward, minAForward + 200);
    drive = constrain(drive, 0, 1023);
    SerialUSB.print("Driving Fore: ");
    SerialUSB.println(drive);
    analogWrite(AIN2, drive);
  }
  if (dir == 0) {
    //SerialUSB.print("Driving ");
    //SerialUSB.println(percent);
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    digitalWrite(AIN2, LOW);
    int drive = map(percent, 49, 0, minABackward, minABackward + 200);
    drive = constrain(drive, 0, 1023);
    SerialUSB.print("Driving Back: ");
    SerialUSB.println(drive);
    analogWrite(AIN1, drive);
  }
  if (percent == 50) {
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
  }
}
  /*
   * Right Motor Control Procedure - 0-49 - Reverse, 51-100 - Forward
   */
void motorB(int percent) {  
  int maxSpeed = 150;
  int minSpeed = 45;
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {
    //SerialUSB.print("Driving ");
    //SerialUSB.println(percent);
    pinMode(BIN2, INPUT);
    pinMode(BIN1, INPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    digitalWrite(BIN2, LOW);
    int drive = map(percent, 51, 100, minBForward, minBForward + 200);
    drive = constrain(drive, 0, 1023);
    analogWrite(BIN1, drive);
    SerialUSB.print("Driving Fore: ");
    SerialUSB.println(drive);
  }
  if (dir == 0) {
    //SerialUSB.print("Driving ");
    //SerialUSB.println(percent);
    pinMode(BIN2, INPUT);
    pinMode(BIN1, INPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(BIN1, LOW);
    int drive = map(percent, 49, 0, minBBackward, minBBackward + 200);
    drive = constrain(drive, 0, 1023);
    analogWrite(BIN2, drive);
    SerialUSB.print("Driving Back: ");
    SerialUSB.println(drive);
  }
  if (percent == 50) {
    pinMode(BIN1, INPUT);
    pinMode(BIN2, INPUT);
  }
}

void MotorTest()
{
  motorA(50);
  motorB(50);
  delay(500);
  digitalWrite(LEDYEL, LOW);
  motorA(100);
  delay(1000);
  motorA(50);
  digitalWrite(LEDYEL, HIGH);
  delay(1000);
  digitalWrite(LEDRED, LOW);
  motorB(100);
  delay(1000);
  motorB(50);
  digitalWrite(LEDRED, HIGH);
  delay(1000);
  digitalWrite(LEDYEL, LOW);
  motorA(1);
  delay(1000);
  motorA(50);
  digitalWrite(LEDYEL, HIGH);
  delay(1000);
  digitalWrite(LEDRED, LOW);
  motorB(1);
  delay(1000);
  motorA(50);
  motorB(50);
  digitalWrite(LEDRED, HIGH);
}

unsigned long PING(int echoPin, int trigPin)  // Sonar sensor procedure.
{
  // Switch signalpin to output
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(trigPin, LOW); // Holdoff
  pinMode(trigPin, INPUT);

  unsigned long echo = pulseIn(echoPin, HIGH, 30000); //Listen for echo
  unsigned long ultrasoundValue = (echo / 58.138);// * .39; //convert to CM then to inches
  return ultrasoundValue;
}

void CalibrateMinMotor() { //Calibrate the minimum voltage to move motor.
  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  attachInterrupt(AENC2, updateEncoderA, CHANGE);
  attachInterrupt(BENC1, updateEncoderB, CHANGE);
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  SerialUSB.println("Calibrating Motors...");
  delay(1000);
  encoderValueA = 0;
  int x = 150;
  while (encoderValueA < 2) {
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, x);
    x = x + 3;
    delay(50);
    //SerialUSB.println(x);
  }
  minAForward = x + 100;
  SerialUSB.print("minAForward: ");
  SerialUSB.println(minAForward);
  x = 150;
  encoderValueA = 0;
  while (encoderValueA > -2) {
    pinMode(AIN1, INPUT);
    pinMode(AIN2, INPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    digitalWrite(AIN2, LOW);
    analogWrite(AIN1, x);
    x = x + 3;
    delay(50);
    //SerialUSB.println(x);
  }
  pinMode(AIN2, INPUT);
  pinMode(AIN1, INPUT);
  minABackward = x + 100;
  SerialUSB.print("minABackward: ");
  SerialUSB.println(minABackward);
  encoderValueB = 0;
  x = 150;
  while (encoderValueB < 2) {
    pinMode(BIN2, INPUT);
    pinMode(BIN1, INPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(BIN2, LOW);
    analogWrite(BIN1, x);
    x = x + 3;
    delay(50);
    //SerialUSB.println(x);
  }
  minBForward = x + 100;
  SerialUSB.print("minBForward: ");
  SerialUSB.println(minBForward);
  x = 150;
  encoderValueB = 0;
  while (encoderValueB > -2) {
    pinMode(BIN2, INPUT);
    pinMode(BIN1, INPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    digitalWrite(BIN1, LOW);
    analogWrite(BIN2, x);
    x = x + 3;
    delay(50);
    //SerialUSB.println(x);
  }
  pinMode(BIN2, INPUT);
  pinMode(BIN1, INPUT);
  minBBackward = x + 100;
  SerialUSB.print("minBBackward: ");
  SerialUSB.println(minBBackward);
  detachInterrupt(AENC1);
  detachInterrupt(AENC2);
  detachInterrupt(BENC1);
  detachInterrupt(BENC2);
}


void updateEncoderA() {
  int MSB = digitalRead(AENC2); //MSB = most significant bit
  int LSB = digitalRead(AENC1); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedA << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b0010 || sum == 0b1101 || sum == 0b1011 || sum == 0b0100) encoderValueA ++;
  if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) encoderValueA --;

  lastEncodedA = encoded; //store this value for next time
}

void updateEncoderB() {
  int MSB = digitalRead(BENC1); //MSB = most significant bit
  int LSB = digitalRead(BENC2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b0010 || sum == 0b1101 || sum == 0b1011 || sum == 0b0100) encoderValueB ++;
  if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) encoderValueB --;

  lastEncodedB = encoded; //store this value for next time
}
