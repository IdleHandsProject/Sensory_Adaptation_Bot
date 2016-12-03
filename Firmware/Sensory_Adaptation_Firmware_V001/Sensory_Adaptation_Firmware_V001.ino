/*I refer to this program as "sensory adaptation" meant for the Sensory Adaptation Robot I built.
   The purpose of this program is to demonstrate the ability to acquire new sensor use not specifically programmed into the Robot.
   The Robot will run a course using the Sharp IR sensors. It will drive in the direction that has the furthest to drive until it has reached a certain distance away from the stop.
   At the same time as determining this distance it will also be recording data from the Ping Sonar Distance sensors and which direction it decided to go. This data does not mean anything at this time.
   The data will be used to train a neural network. Once 6 directions have been done in the course, it will then train the network with its new data. At this time the robot should be placed at the beginning of the course.
   However this is not necessary, it will work anywhere. You can also demonstate that the IR sensors are not longer working by removing their power with the jumper.
   It has successfully demonstated that the neural network allows new sensors to be adapted for use in place of other sensors.

   Program pieced together by other programs by Sean Hodgins
   This is under Creative Commons Share Alike 3.0 - Open Source.
*/

#include <math.h>
////////LEDs
#define LEDYEL 25
#define LEDRED 26

///////SHARP IR DISTANCE SENSORS
#define IR1 A2
#define IR2 A3
#define IR3 A1

////////MOTORS ENCODERS AND CONTROL
#define AENC1 A4
#define AENC2 A5
#define BENC1 10
#define BENC2 12

#define AIN1 45
#define AIN2 44
#define BIN1 9
#define BIN2 8

///////SONAR PING SENSORS
#define TRIG1 33
#define ECHO1 32
#define TRIG2 7
#define ECHO2 6
#define TRIG3 4
#define ECHO3 3

////////BUTTONS
#define BUT_RED 11
#define BUT_BLUE 13

/////ENCODER VARS
volatile int lastEncodedA = 0;
volatile long encoderValueA = 0;
long lastencoderValueA = 0;

volatile int lastEncodedB = 0;
volatile long encoderValueB = 0;
long lastencoderValueB = 0;
//int lastMSBA = 0;
//int lastLSBA = 0;

/////////NEURAL NET CONFIG
const int PatternCount = 4;
const int InputNodes = 3;
const int HiddenNodes = 4;
const int OutputNodes = 3;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

float Input[PatternCount][InputNodes] = {
  { 1, 0, 0 },  // 0 RIGHT
  { 0, 1, 0 },  // 1 STRAIGHT
  { 0, 0, 1 },  // 2 LEFT
  { 0, 0, 0 },  // 3 BACK
};

const byte Target[PatternCount][OutputNodes] = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 },
  { 0, 0, 0 },
};

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

/////////////////////////

int BUT_REDstate = 0;

void setup() {

  //
  pinMode(AIN1, OUTPUT);
  digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN2, LOW);
  pinMode(BIN1, OUTPUT);
  digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT);
  digitalWrite(BIN2, LOW);
  motorA(0, 0);
  motorB(0, 0);

  pinMode(AENC1, INPUT);
  pinMode(AENC2, INPUT);
  pinMode(BENC1, INPUT);
  pinMode(BENC2, INPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);

  pinMode(BUT_RED, INPUT_PULLUP);
  //digitalWrite(BUT_RED, HIGH);
  pinMode(BUT_BLUE, INPUT_PULLUP);
  //digitalWrite(BUT_BLUE, HIGH);


  randomSeed(analogRead(A0));
  ReportEvery1000 = 1;
  for ( p = 0 ; p < PatternCount ; p++ ) {
    RandomizedIndex[p] = p ;
  }
  analogReadResolution(12);
  SerialUSB.begin(115200);
  Serial1.begin(115200); //Bluetooth Serial
  Serial1.println("Enter a character to start!");
  BUT_REDstate = digitalRead(BUT_RED);
  while (Serial1.available() == 0 && SerialUSB.available() == 0 && BUT_REDstate == HIGH) {
    BUT_REDstate = digitalRead(BUT_RED); //Wait for input from either Serial
  }
  Serial1.read();
  while (BUT_REDstate == LOW) {
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
  }
  /*int BUT_REDstate = digitalRead(BUT_RED);
    while (BUT_REDstate == HIGH) {
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
    }
    SerialUSB.println("Button Pressed");
    while (BUT_REDstate == LOW) {
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
    }
    SerialUSB.println("Button Released");
  */
  motorA(10, 1);
  motorB(10, 1);
  delay(50);
  motorA(0, 0);
  motorB(0, 0);
  delay(2000);

  /*ChangeDir(0);
    delay(1000);
    ChangeDir(1);
    delay(1000);
    ChangeDir(2);
    delay(1000);
    ChangeDir(3);
    while(1);
  */

}

void loop() {

  int Trial = 0;
  while (Error < Success) { ////AFTER NETWORK TRAINED CONTINUE TO RUN BASED ON TRAINED NETWORK

    int TempInt;
    float TestInput[] = {0, 0, 0};
    int ping1 = PING(ECHO1, TRIG1);
    delay(10);
    int ping2 = PING(ECHO2, TRIG2);
    delay(10);
    int ping3 = PING(ECHO3, TRIG3);

    TempInt = map(ping1, 5, 62, 0, 100000);
    TestInput[0] = TempInt / 100000.000000;
    TempInt = map(ping2, 5, 62, 0, 100000);
    TestInput[1] = TempInt / 100000.000000;
    TempInt = map(ping3, 5, 62, 0, 100000);
    TestInput[2] = TempInt / 100000.000000;

    Serial1.println("Testing NN: ");
    int dir = InputToOutput(TestInput[0], TestInput[1], TestInput[2]);
    ChangeDir(dir);
    delay(2000);
    DriveStraightStop();
    delay(1000);
    Trial++;
    if (Trial > 5) {
      digitalWrite(LEDRED, HIGH);
      Serial1.println("Send another char to test network again");
      while (Serial1.available() == 0 && BUT_REDstate == HIGH) {
        BUT_REDstate = digitalRead(BUT_RED); //Wait for input from either Serial
      }
      Serial1.read();
      while (BUT_REDstate == LOW) {
        BUT_REDstate = digitalRead(BUT_RED);
        delay(50);
      }
      digitalWrite(LEDRED, LOW);
      Trial = 0;
      motorA(10, 1);
  motorB(10, 1);
  delay(50);
  motorA(0, 0);
  motorB(0, 0);
  delay(2000);
    }

  }
  /*int ping1 = PING(ECHO1, TRIG1);
    int ping2 = PING(ECHO2, TRIG2);
    int ping3 = PING(ECHO3, TRIG3);
    SerialUSB.print("Ping1: ");
    SerialUSB.print(ping1);
    SerialUSB.print(", Ping2: ");
    SerialUSB.print(ping2);
    SerialUSB.print(", Ping3: ");
    SerialUSB.println(ping3);
    float ReadIR1 = analogRead(IR1);
    float ReadIR2 = analogRead(IR2);
    float ReadIR3 = analogRead(IR3);
    SerialUSB.print("IR1: ");
    SerialUSB.print(ReadIR1);
    SerialUSB.print(", IR2: ");
    SerialUSB.print(ReadIR2);
    SerialUSB.print(", IR3: ");
    SerialUSB.println(ReadIR3);
    delay(1000);*/
  Serial1.println("Original Training Data:");
  for (int x = 0; x < 4; x++) {
    for (int y = 0; y < 3; y++) {
      Serial1.print(Input[x][y]);
      Serial1.print(",");
    }
    Serial1.println();
  }
  for (int Move = 0; Move < 6; Move++) {
    int dir = CheckDirTrainAdd();
    ChangeDir(dir);
    delay(1000);
    DriveStraightStop();
  }

  Serial1.println("New Training Data:");
  for (int x = 0; x < 4; x++) {
    for (int y = 0; y < 3; y++) {
      Serial1.print(Input[x][y]);
      Serial1.print(",");
    }
    Serial1.println();
  }
  Serial1.println("Finished.");
  Serial1.println("Training Network.");
  delay(1000);
  while (TrainANN() == 0);

  Serial1.println("Place robot in initial position, and send another Char to test neural network");
  delay(500);
  digitalWrite(LEDRED, HIGH);
  delay(500);
  while (Serial1.available() == 0  && BUT_REDstate == HIGH) {
    BUT_REDstate = digitalRead(BUT_RED); //Wait for input from either Serial
  }
  Serial1.read();
  while (BUT_REDstate == LOW) {
    BUT_REDstate = digitalRead(BUT_RED);
    delay(50);
  }
  digitalWrite(LEDRED, LOW);
  motorA(10, 1);
  motorB(10, 1);
  delay(50);
  motorA(0, 0);
  motorB(0, 0);
  delay(2000);
}



void updateEncoderA() {
  int MSB = digitalRead(AENC1); //MSB = most significant bit
  int LSB = digitalRead(AENC2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedA << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1100 || sum == 0b1111) encoderValueA ++;
  if (sum == 0b1101 || sum == 0b0110) encoderValueA --;

  lastEncodedA = encoded; //store this value for next time
}

void updateEncoderB() {
  int MSB = digitalRead(BENC1); //MSB = most significant bit
  int LSB = digitalRead(BENC2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1100 || sum == 0b1111) encoderValueB ++;
  if (sum == 0b1101 || sum == 0b0110) encoderValueB --;

  lastEncodedB = encoded; //store this value for next time
}

int ChangeDir(int dir) {
  if (dir == 0) {
    TurnRightStop();
  }
  if (dir == 2) {
    TurnLeftStop();
  }
  if (dir == 3) {
    TurnLeftStop();
    TurnLeftStop();
  }
}

//Percent of motor speec and direction(1/0, forward/backward);
void motorA(int percent, int dir) {
  if (dir == 1) {
    digitalWrite(AIN1, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(AIN2, drive);
  }
  if (dir == 0) {
    digitalWrite(AIN2, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(AIN1, drive);
  }
  if (percent < 1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void motorB(int percent, int dir) {
  if (dir == 0) {
    digitalWrite(BIN1, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(BIN2, drive);
  }
  if (dir == 1) {
    digitalWrite(BIN2, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(BIN1, drive);
  }
  if (percent < 1) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

int CheckDirTrainAdd() { //0 = RIGHT, 1 = STRAIGHT, 2 = LEFT 3 = BACK

  int dir;
  float Train[InputNodes];
  int TrainInt[InputNodes];
  int ping1 = PING(ECHO1, TRIG1);
  delay(10);
  int ping2 = PING(ECHO2, TRIG2);
  delay(10);
  int ping3 = PING(ECHO3, TRIG3);
  int ReadIR1 = avgIR(IR1);
  int ReadIR2 = avgIR(IR2);
  int ReadIR3 = avgIR(IR3);

  Serial1.print("Right Sensor: ");
  Serial1.println(ReadIR1);
  Serial1.print("Middle Sensor: ");
  Serial1.println(ReadIR2);
  Serial1.print("Left Sensor: ");
  Serial1.println(ReadIR3);
  delay(500);
  int minimum = min3(ReadIR1, ReadIR2, ReadIR3);
  if (minimum == ReadIR3) {
    dir = 2; //LEFT
    Serial1.println("Going Left!");
  }
  if (minimum == ReadIR1) {
    dir = 0; //RIGHT
    Serial1.println("Going Right!");
  }
  if (minimum == ReadIR2) {
    dir = 1; //STRAIGHT
    Serial1.println("Going Straight!");
  }
  if (minimum > 2500) {
    dir = 3; //TURN AROUND
    Serial1.println("Going Back!");
  }

  TrainInt[0] = map(ping1, 5, 62, 0, 100000);
  Train[0] = TrainInt[0] / 100000.000000;

  TrainInt[1] = map(ping2, 5, 62, 0, 100000);
  Train[1] = TrainInt[1] / 100000.000000;

  TrainInt[2] = map(ping3, 5, 62, 0, 100000);
  Train[2] = TrainInt[2] / 100000.000000;

  for (int x = 0; x < InputNodes; x++) {
    Input[dir][x] = Train[x];
  }

  return dir;
}

float avgIR(int IR) {
  float avg = 0;
  float numavg = 100;
  for (int x = 0; x < numavg; x++) {
    avg = analogRead(IR) + avg;
  }
  avg = avg / numavg;
  return avg;
}

int min3(int a, int b, int c)
{
  int minguess;

  minguess = min(a, b);
  minguess = min(minguess, c);

  return (minguess);
}

float max3(float a, float b, float c)
{
  float maxguess;

  maxguess = max(a, b);
  maxguess = max(maxguess, c);
  Serial1.print("Finding Max of: ");
  Serial1.print(a);
  Serial1.print(", ");
  Serial1.print(b);
  Serial1.print(", ");
  Serial1.println(c);
  Serial1.print("Max: ");
  Serial1.println(maxguess);
  return (maxguess);
}

void DriveStraightStop() {
  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  //attachInterrupt(AENC2, updateEncoderA, CHANGE);
  //attachInterrupt(BENC1, updateEncoderB, CHANGE);
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  int distance = PING(ECHO2, TRIG2);

  int mA = 25;
  int mB = 25;
  int mRateA = 250;
  int mRateB = 250;
  encoderValueA = 0;
  encoderValueB = 0;
  while (distance > 7) {
    motorA(mA, 1);
    motorB(mB, 1);
    int right = PING(ECHO1, TRIG1);
    delay(5);
    int left = PING(ECHO3, TRIG3);
    delay(5);
    distance = PING(ECHO2, TRIG2);
    delay(10);
    
    if (encoderValueA > (encoderValueB)) {
      mRateA--;
      mRateB++;
    }
    if ((encoderValueA) < encoderValueB) {
      mRateB--;
      mRateA++;
    }

    /*if (left > right) {
      mRateA--;
      mRateB++;
      }
      if (left < right) {
      mRateB--;
      mRateA++;
      }*/

    mA = mRateA / 10;
    mB = mRateB / 10;
    mA = constrain(mA, 0, 35);
    mB = constrain(mB, 0, 35);
    //SerialUSB.print(mRateA);
    //SerialUSB.print(", ");
    //SerialUSB.println(mRateB);
    //SerialUSB.print(encoderValueA);
    //SerialUSB.print(", ");
    //SerialUSB.println(encoderValueB);
    //delay(50);
    
  }
  motorA(0, 0);
  motorB(0, 0);
}

void TurnLeftStop() {
  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  //attachInterrupt(AENC2, updateEncoderA, CHANGE);
  //attachInterrupt(BENC1, updateEncoderB, CHANGE);
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  encoderValueA = 0;
  encoderValueB = 0;
  while (encoderValueA > -73 || encoderValueB < 70) {
    if (encoderValueA > -73) {
      motorA(10, 0);
    }
    else {
      motorA(0, 0);
    }
    if (encoderValueB < 70) {
      motorB(10, 1);
    }
    else {
      motorB(0, 0);
    }
    //SerialUSB.print(encoderValueA);
    //SerialUSB.print(", ");
    //SerialUSB.println(encoderValueB);
  }
  Serial1.print("Encoders: ");
  Serial1.print(encoderValueA);
  Serial1.print(", ");
  Serial1.println(encoderValueB);
  motorA(0, 0);
  motorB(0, 0);
}

void TurnRightStop() {
  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  //attachInterrupt(AENC2, updateEncoderA, CHANGE);
  //attachInterrupt(BENC1, updateEncoderB, CHANGE);
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  encoderValueA = 0;
  encoderValueB = 0;
  while (encoderValueB > -77 || encoderValueA < 70) {
    if (encoderValueB > -77) {
      motorB(10, 0);
    }
    else {
      motorB(0, 0);
    }
    if (encoderValueA < 70) {
      motorA(10, 1);
    }
    else {
      motorA(0, 0);
    }
    //SerialUSB.print(encoderValueA);
    //SerialUSB.print(", ");
    //SerialUSB.println(encoderValueB);
  }

  Serial1.print("Encoders: ");
  Serial1.print(encoderValueA);
  Serial1.print(", ");
  Serial1.println(encoderValueB);
  motorA(0, 0);
  motorB(0, 0);
}

int getRPM_A() {
  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  attachInterrupt(AENC2, updateEncoderA, CHANGE);
  int encoderA_S = 0;
  int encoderA_F = 0;
  encoderA_S = encoderValueA;
  delay(10);
  encoderA_F = encoderValueA;
  int change = encoderA_F - encoderA_S;
  int rpm = ((change / 12) * 100 * 60) / 26;
  return change;
  detachInterrupt(AENC1);
  detachInterrupt(BENC1);
}

void motorA_RPM(int rpm, int dir) {

  attachInterrupt(AENC1, updateEncoderA, CHANGE);
  attachInterrupt(AENC2, updateEncoderA, CHANGE);
  int timeS, timeE = 0;
  int encoderA_S = 0;
  int encoderA_F = 0;


  encoderA_S = encoderValueA;
  delay(10);


}


unsigned long PING(int echoPin, int trigPin)
{
  // Switch signalpin to output
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(trigPin, LOW); // Holdoff
  pinMode(trigPin, INPUT);
  //pinMode(trigpin, HIGH);
  // Turn on pullup resistor
  // please note that pulseIn has a 1sec timeout, which may
  // not be desirable. Depending on your sensor specs, you
  // can likely bound the time like this -- marcmerlin
  // echo = pulseIn(ultraSoundSignal, HIGH, 38000)
  unsigned long echo = pulseIn(echoPin, HIGH, 30000); //Listen for echo
  unsigned long ultrasoundValue = (echo / 58.138);// * .39; //convert to CM then to inches
  return ultrasoundValue;
}


byte TrainANN() {

  byte Trained = 0;
  /******************************************************************
    Initialize HiddenWeights and ChangeHiddenWeights
  ******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    for ( j = 0 ; j <= InputNodes ; j++ ) {
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  /******************************************************************
    Initialize OutputWeights and ChangeOutputWeights
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i ++ ) {
    for ( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  SerialUSB.println("Initial/Untrained Outputs: ");
  Serial1.println("Initial/Untrained Outputs: ");
  toTerminal();
  /******************************************************************
    Begin training
  ******************************************************************/

  for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

    /******************************************************************
      Randomize order of training patterns
    ******************************************************************/
    digitalWrite(LEDYEL, HIGH);
    for ( p = 0 ; p < PatternCount ; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p] ;
      RandomizedIndex[p] = RandomizedIndex[q] ;
      RandomizedIndex[q] = r ;
    }
    Error = 0.0 ;
    digitalWrite(LEDYEL, LOW);
    /******************************************************************
      Cycle through each training pattern in the randomized order
    ******************************************************************/
    for ( q = 0 ; q < PatternCount ; q++ ) {
      p = RandomizedIndex[q];

      /******************************************************************
        Compute hidden layer activations
      ******************************************************************/
      digitalWrite(LEDRED, HIGH);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = HiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[p][j] * HiddenWeights[j][i] ;
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
      }
      digitalWrite(LEDRED, LOW);
      /******************************************************************
        Compute output layer activations and calculate errors
      ******************************************************************/
      digitalWrite(LEDYEL, HIGH);
      for ( i = 0 ; i < OutputNodes ; i++ ) {
        Accum = OutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
      }
      digitalWrite(LEDYEL, LOW);
      /******************************************************************
        Backpropagate errors to hidden layer
      ******************************************************************/
      digitalWrite(LEDRED, HIGH);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = 0.0 ;
        for ( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }
      digitalWrite(LEDRED, LOW);

      /******************************************************************
        Update Inner-->Hidden Weights
      ******************************************************************/

      digitalWrite(LEDYEL, HIGH);
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }
      digitalWrite(LEDYEL, LOW);
      /******************************************************************
        Update Hidden-->Output Weights
      ******************************************************************/
      digitalWrite(LEDRED, HIGH);
      for ( i = 0 ; i < OutputNodes ; i ++ ) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }
      digitalWrite(LEDRED, LOW);
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

      Serial1.println();
      Serial1.println();
      Serial1.print ("TrainingCycle: ");
      Serial1.print (TrainingCycle);
      Serial1.print ("  Error = ");
      Serial1.println (Error, 5);

      toTerminal();
      delay(500);
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

  Serial1.println ();
  Serial1.println();
  Serial1.print ("TrainingCycle: ");
  Serial1.print (TrainingCycle);
  Serial1.print ("  Error = ");
  Serial1.println (Error, 5);

  toTerminal();
  delay(500);
  SerialUSB.println ();
  SerialUSB.println ();
  SerialUSB.println ("Training Set Solved! ");
  SerialUSB.println ("--------");
  SerialUSB.println ();
  SerialUSB.println ();

  Serial1.println ();
  Serial1.println ();
  Serial1.println ("Training Set Solved! ");
  Serial1.println ("--------");
  Serial1.println ();
  Serial1.println ();

  ReportEvery1000 = 1;
  Trained = 1;
  return Trained;
}

void toTerminal()
{

  for ( p = 0 ; p < PatternCount ; p++ ) {
    SerialUSB.println();
    SerialUSB.print ("  Training Pattern: ");
    SerialUSB.println (p);
    SerialUSB.print ("  Input ");

    Serial1.println();
    Serial1.print ("  Training Pattern: ");
    Serial1.println (p);
    Serial1.print ("  Input ");

    for ( i = 0 ; i < InputNodes ; i++ ) {
      SerialUSB.print (Input[p][i], DEC);
      SerialUSB.print (" ");

      Serial1.print (Input[p][i], DEC);
      Serial1.print (" ");
    }
    SerialUSB.print ("  Target ");
    Serial1.print ("  Target ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      SerialUSB.print (Target[p][i], DEC);
      SerialUSB.print (" ");
      Serial1.print (Target[p][i], DEC);
      Serial1.print (" ");
    }
    /******************************************************************
      Compute hidden layer activations
    ******************************************************************/
    digitalWrite(LEDYEL, HIGH);
    for ( i = 0 ; i < HiddenNodes ; i++ ) {
      Accum = HiddenWeights[InputNodes][i] ;
      for ( j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[p][j] * HiddenWeights[j][i] ;
      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }
    digitalWrite(LEDYEL, LOW);
    /******************************************************************
      Compute output layer activations and calculate errors
    ******************************************************************/
    digitalWrite(LEDRED, HIGH);
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Accum = OutputWeights[HiddenNodes][i] ;
      for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }
    digitalWrite(LEDRED, LOW);
    SerialUSB.print ("  Output ");
    Serial1.print ("  Output ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      SerialUSB.print (Output[i], 5);
      SerialUSB.print (" ");
      Serial1.print (Output[i], 5);
      Serial1.print (" ");
    }
  }


}

int InputToOutput(float In1, float In2, float In3)   //USED TO CHECK THE OUTPUT FROM A GIVEN INPUT
{
  float TestInput[] = {0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;


  Serial1.println();
  Serial1.print (" Test Pattern: ");
  Serial1.println ("Test");
  Serial1.print ("  Input ");
  for ( i = 0 ; i < InputNodes ; i++ ) {
    Serial1.print (TestInput[i], DEC);
    Serial1.print (" ");
  }
  //Serial1.print ("  Target ");
  //for( i = 0 ; i < OutputNodes ; i++ ) {
  //  Serial1.print (Target[p][i], DEC);
  //  Serial1.print (" ");
  //}
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
  Serial1.print ("  Output ");
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Serial1.print (Output[i], 5);
    Serial1.print (" ");
  }
  int dir = 0;
  float maximum = max3(Output[0], Output[1], Output[2]);
  Serial1.print("Max is: ");
  Serial1.println(maximum);
  if (maximum == Output[2]) {
    dir = 2; //LEFT
    Serial1.println("Going Left!");
  }
  if (maximum == Output[0]) {
    dir = 0; //RIGHT
    Serial1.println("Going Right!");
  }
  if (maximum == Output[1]) {
    dir = 1; //STRAIGHT
    Serial1.println("Going Straight!");
  }
  if (maximum < 0.1) {
    dir = 3; //TURN AROUND
    Serial1.println("Going Back!");
  }
  return dir;
}

