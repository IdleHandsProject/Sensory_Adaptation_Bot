#define LED 1
//SHARP IR DISTANCE SENSORS
#define IR1 A1
#define IR2 A3
#define IR3 A2

//MOTORS ENCODERS AND CONTROL
#define AENC1 A4
#define AENC2 A5
#define BENC1 10
#define BENC2 12

#define AIN1 45
#define AIN2 44
#define BIN1 9
#define BIN2 8

//SONAR PING SENSORS
#define TRIG1 4
#define ECHO1 3
#define TRIG2 7
#define ECHO2 6
#define TRIG3 33
#define ECHO3 32

//BUTTONS
#define BUT_RED 11
#define BUT_BLUE 13


volatile int lastEncodedA = 0;
volatile long encoderValueA = 0;
long lastencoderValueA = 0;

volatile int lastEncodedB = 0;
volatile long encoderValueB = 0;
long lastencoderValueB = 0;
//int lastMSBA = 0;
//int lastLSBA = 0;

void setup() {

  pinMode(AIN1, OUTPUT);
  digitalWrite(AIN1,LOW);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1,LOW);
  pinMode(BIN1, OUTPUT);
  digitalWrite(BIN1,LOW);
  pinMode(BIN2, OUTPUT);
  digitalWrite(BIN2,LOW);
  motorA(0,0);
  motorB(0,0);
  
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

  

  
SerialUSB.begin(115200);
//while(!SerialUSB);
int BUT_REDstate = digitalRead(BUT_RED);
while(BUT_REDstate == HIGH){
  BUT_REDstate = digitalRead(BUT_RED);
  delay(50);
}
SerialUSB.println("Button Pressed");
while(BUT_REDstate == LOW){
  BUT_REDstate = digitalRead(BUT_RED);
  delay(50);
}
SerialUSB.println("Button Released");
motorA(10,1);
motorB(10,1);
delay(50);
motorA(0,0);
motorB(0,0);
delay(3000);


}

void loop() {

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
DriveStraightStop();
delay(1000);
TurnLeftStop();
delay(1000);
TurnLeftStop();
delay(1000);
DriveStraightStop();
while(1);

}



void updateEncoderA(){
  int MSB = digitalRead(AENC1); //MSB = most significant bit
  int LSB = digitalRead(AENC2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedA << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1100 || sum == 0b1111) encoderValueA ++;
  if(sum == 0b1101 || sum == 0b0110) encoderValueA --;

  lastEncodedA = encoded; //store this value for next time
}

void updateEncoderB(){
  int MSB = digitalRead(BENC1); //MSB = most significant bit
  int LSB = digitalRead(BENC2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1100 || sum == 0b1111) encoderValueB ++;
  if(sum == 0b1101 || sum == 0b0110) encoderValueB --;

  lastEncodedB = encoded; //store this value for next time
}


//Percent of motor speec and direction(1/0, forward/backward);
void motorA(int percent, int dir){
  if (dir==1){
    digitalWrite(AIN1, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(AIN2, drive);
  }
  if (dir==0){
    digitalWrite(AIN2, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(AIN1, drive);
  }
  if (percent<1){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}

void motorB(int percent, int dir){
  if (dir==0){
    digitalWrite(BIN1, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(BIN2, drive);
  }
  if (dir==1){
    digitalWrite(BIN2, LOW);
    int drive = map(percent, 0, 100, 170, 255);
    analogWrite(BIN1, drive);
  }
  if (percent<1){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
}

void DriveStraightStop(){
  attachInterrupt(AENC1, updateEncoderA, CHANGE); 
  //attachInterrupt(AENC2, updateEncoderA, CHANGE);
  //attachInterrupt(BENC1, updateEncoderB, CHANGE); 
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  int distance = PING(ECHO2, TRIG2);
  
  int mA = 15;
  int mB = 15;
  int mRateA = 150;
  int mRateB = 150;
  encoderValueA = 0;
  encoderValueB = 0;
  while(distance > 10){
    motorA(mA, 1);
    motorB(mB, 1);
    if (encoderValueA > encoderValueB){
      mRateA--;
      mRateB++;
    }
    if (encoderValueA < encoderValueB){
      mRateB--;
      mRateA++;
    }
    mA = mRateA / 10;
    mB = mRateB / 10;
    mA = constrain(mA,0,25);
     mB = constrain(mB,0,25);
     //SerialUSB.print(mRateA);
     //SerialUSB.print(", ");
     //SerialUSB.println(mRateB);
     SerialUSB.print(encoderValueA);
     SerialUSB.print(", ");
     SerialUSB.println(encoderValueB);
    delay(50);
    distance = PING(ECHO2,TRIG2);
  }
  motorA(0,0);
  motorB(0,0);
}

void TurnLeftStop(){
  attachInterrupt(AENC1, updateEncoderA, CHANGE); 
  //attachInterrupt(AENC2, updateEncoderA, CHANGE);
  //attachInterrupt(BENC1, updateEncoderB, CHANGE); 
  attachInterrupt(BENC2, updateEncoderB, CHANGE);
  encoderValueA = 0;
  encoderValueB = 0;
  while(encoderValueA > -60 || encoderValueB < 70){
  if (encoderValueA > -60){
   motorA(15, 0); 
  }
  else{
    motorA(0,0);
  }
  if (encoderValueB < 70){
   motorB(15, 1); 
  }
  else{
    motorB(0,0);
  }
  SerialUSB.print(encoderValueA);
     SerialUSB.print(", ");
     SerialUSB.println(encoderValueB);
  }
  motorA(0,0);
  motorB(0,0);
}

int getRPM_A(){
  attachInterrupt(AENC1, updateEncoderA, CHANGE); 
  attachInterrupt(AENC2, updateEncoderA, CHANGE);
  int encoderA_S = 0;
  int encoderA_F = 0;
  encoderA_S = encoderValueA;
  delay(10);
  encoderA_F = encoderValueA;
  int change = encoderA_F - encoderA_S;
  int rpm = ((change/12) * 100 * 60)/26;
  return change;
  detachInterrupt(AENC1);
  detachInterrupt(BENC1);
}

void motorA_RPM(int rpm, int dir){

  attachInterrupt(AENC1, updateEncoderA, CHANGE); 
  attachInterrupt(AENC2, updateEncoderA, CHANGE);
  int timeS,timeE = 0;
  int encoderA_S = 0;
  int encoderA_F = 0;
  
  
  encoderA_S = encoderValueA;
  delay(10);

  
}


unsigned long PING(int echoPin, int trigPin)
{ 
   // Switch signalpin to output
  pinMode(trigPin,OUTPUT);
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
  unsigned long echo = pulseIn(echoPin, HIGH,30000); //Listen for echo
  unsigned long ultrasoundValue = (echo / 58.138);// * .39; //convert to CM then to inches
  return ultrasoundValue;
  
}
