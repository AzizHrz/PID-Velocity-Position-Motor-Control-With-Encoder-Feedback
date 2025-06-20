#include <Arduino.h>
#include <util/atomic.h>

// variabl for PID encoder motor control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

// setPoint
int NbTr = 0;
int target = 0;
int realTraget = 0;

// PID gains and computation for encoder
//  float motorKp=2.0;
float turnKp = 1;
// float motorKd=0.2;
float turnKd = 0.1;
float turnKi = 0.001;
void turnLeft();
void turnRight();
float pidController(int , float , float , float, int );
void serialPLotterMotor(int) ;

//affichage capteurs et vitesse moteurs
uint16_t tEncoder =0;
uint16_t previoustEncoder =0;
const uint16_t displaytEncoder = 500; /// temp d'affichage de la fonction display

// Pinsettings ======================================================================
#define ENCA 3
#define ENCB A0//   

#define ENCAR 2
#define ENCBR A1
#define SPEEDPIN_R 9
#define IN1 A3
#define IN2 A2
#define IN3 A4
#define IN4 A5
#define SPEEDPIN_L 10
//
unsigned int pwmLeft, pwmRight;
int Left, Right;

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_iR = 0;
volatile float velocity_iR = 0;
volatile long prevT_iR = 0;

float v1FiltR = 0;
float v1PrevR = 0;
float v2FiltR = 0;
float v2PrevR = 0;

float eintegralR = 0;

void setMotor(int , int , int , int , int );
void readEncoder();
void readEncoderR();
void pidvelocity(float, float);
void stop();

void setup() {

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  delay(500);
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  
  Serial.begin(115200); 
  //while (!Serial) {}
	Serial.println("hellllooooooo");

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(SPEEDPIN_L, OUTPUT);
	pinMode(SPEEDPIN_R, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAR),
                  readEncoderR,RISING);              
		
	stop();
	delay(1000);
   Serial.println("Started !!");
  
}
//affichage
uint16_t t =0;
uint16_t previoust = 0 ;
const uint16_t displayt = 200;
int k=30000;
int counter=0;
bool test =1;
void loop() {
// while (k>0)
// {
//     k++;
//     Serial.print(">counter:");
//     Serial.println(k);
// }
//    if(test){
//     test=0;
//     pos_i=0; turnRight();
//    }
//     stop();
   // pos_i=0; turnLeft();
   // pos_i=0; pidvelocity(50,50);

    stop();
    counter++;
    Serial.print(">counter:");
    Serial.println(counter);
    int t=millis();
    //int pt=0;
    if(counter%2==0 && counter<16) {
        delay(2000);
        stop();
    }else if(counter==1){
        //while(distancef<5)
        v2Filt=0;
            v2FiltR=0;
        while(millis()-t <2000){
            pidvelocity(80,80);
        }
    }else if(counter==3){
        pos_i=0; pos_iR=0; turnRight();
    }else if(counter==5){
        v2Filt=0;
            v2FiltR=0;
        while(millis()- t <2000){
            pidvelocity(80,80);
        }
    }else if(counter==7){
        pos_i=0; pos_iR=0; 
        turnRight();
    }else if(counter==9){
        v2Filt=0;
            v2FiltR=0;
        while(millis()- t <2000){
            pidvelocity(80,80);
        }
    }else if(counter==11){
        pos_i=0; pos_iR=0; turnLeft();
    }else if(counter==13){
        v2Filt=0;
            v2FiltR=0;
        while(millis()- t<2000){
            pidvelocity(80,80);
        }
    }else if(counter==15){
        pos_i=0; pos_iR=0; turnLeft();
    }

}    
void pidvelocity(float vt, float vtR){
  // read the position in an atomic block
  // to avoid potential misreads
  //int pos = 0;
  
  float velocity2 = 0;
  float velocity2R = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    //pos = pos_i;
    velocity2 = velocity_i;
    //posR = pos_iR;
    velocity2R = velocity_iR;
  }

  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  prevT = currT;

  // Convert count/s to RPM
  float v2 = velocity2/600.0*60.0;
  float v2R = velocity2R/600.0*60.0;
  //float v2 = velocity2/207.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;
  v2FiltR = 0.854*v2FiltR + 0.0728*v2R + 0.0728*v2PrevR;
  v2PrevR = v2R;
  
 // float vt =40;
  //float vtR =40;
  float e = vt-v2Filt;
  eintegral = eintegral + e*deltaT;
  float eR = vtR-v2FiltR;
  eintegralR = eintegralR + eR*deltaT;


 // Compute the control signal u
  float kp = 7.5;
  float ki = 0.1;
// Compute the control signal u
  float kpR = 7.5;
  float kiR = 0.1;

  float u = kp*e + ki*eintegral;
  float uR = kpR*eR + kiR*eintegralR;
  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,SPEEDPIN_L,IN4,IN3);
  // Set the motor speed and direction
  int dirR = 1;
  if (uR<0){
    dirR = -1;
  }
  int pwrR = (int) fabs(uR);
  if(pwrR > 150){
    pwrR = 150;
  }
  setMotor(dirR,pwrR,SPEEDPIN_R,IN1,IN2);


  t = millis();
if((t-previoust) > displayt){
  Serial.print(">velocity1R:");
  Serial.println(velocity2);
  Serial.print(">RPM:");
  Serial.println(v2);  
  Serial.print(">v2Filt:");
  Serial.println(v2Filt);
  Serial.print(">erreur:");
  Serial.println(e);
  Serial.print(">PID:");
  Serial.println(u);
  Serial.print(">targets:  " );
  Serial.println(vt);
previoust = t ;
}

  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
  analogWrite(pwm,pwmVal); // Motor speed
}
void stop(){
    setMotor(0, 0, SPEEDPIN_R, IN1, IN2);
    setMotor(0, 0, SPEEDPIN_L, IN4, IN3);
    
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = -1;
  }
  else{
    // Otherwise, increment backward
    increment = 1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

void readEncoderR(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCBR);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_iR = pos_iR + increment;

  // Compute velocity with method 2
  long currTR = micros();
  float deltaTR= ((float) (currTR - prevT_iR))/1.0e6;
  velocity_iR = increment/deltaTR;
  prevT_iR = currTR;
}

void turnLeft()
{
    int encoderCount =0;
    Serial.print("ydourrrrrrrrrr left");
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // pos = pos_i;
        //   velocity2 = velocity_i;
        encoderCount = pos_iR;
        // velocity2R = velocity_iR;
    }
    NbTr = 2;
    target = NbTr * 207;
    realTraget = encoderCount + target;

    do
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // pos = pos_i;
        //   velocity2 = velocity_i;
        encoderCount = pos_iR;
        // velocity2R = velocity_iR;
    }
        float u = pidController(realTraget, turnKp, turnKd, turnKi, encoderCount); // i change target by target - encoderCounter

        // moveMotorA(u);
        // moveMotor (In1, In2, EnA, u);
        // move2Motor(In1, In2, In3, In4, EnA, EnB, u);
        // Set the motor speed and direction
        int dirR = 1;
        if (u < 0)
        {
            dirR = -1;
        }
        int pwrR = (int)fabs(u);
        if (pwrR > 255)
        {
            pwrR = 255;
        }
        setMotor(dirR, pwrR, SPEEDPIN_R, IN2, IN1);
        setMotor(0, 0, SPEEDPIN_L, IN4, IN3);
        serialPLotterMotor(encoderCount);

    } while (!(realTraget <= encoderCount));
}
void turnRight()
{   
    int encoderCount =0;
    Serial.print("ydourrrrrrrrrr right");
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        encoderCount = pos_i;
        //  velocity2 = velocity_i;
        // encouderCount = pos_iR;
        // velocity2R = velocity_iR;
    }
    NbTr = 2;
    target = NbTr * 207;
    realTraget = encoderCount + target;

    do
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            encoderCount = pos_i;
            //  velocity2 = velocity_i;
            // encouderCount = pos_iR;
            // velocity2R = velocity_iR;
        }

        float u = pidController(realTraget, turnKp, turnKd, turnKi, encoderCount); // i change target by target - encoderCounter

        // moveMotorA(u);
        // moveMotor (In1, In2, EnA, u);
        // move2Motor (In1, In2, In3, In4, EnA, EnB, u);
        // Set the motor speed and direction
        int dir = 1;
        if (u < 0)
        {
            dir = -1;
        }
        int pwr = (int)fabs(u);
        if (pwr > 255)
        {
            pwr = 255;
        }
        setMotor(dir, pwr, SPEEDPIN_L, IN3, IN4);
        setMotor(0, 0, SPEEDPIN_R, IN1, IN2);
        serialPLotterMotor(encoderCount);

    } while (!(realTraget <= encoderCount));
}

float pidController(int target, float Kp, float Kd, float Ki, int encoderCount)
{
    // mesure the time elapsed since the last iteration
    long currentTime = micros();
    float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

    // compte the error, derivative and integral
    int e = encoderCount - target;
    float eDirivative = (e - ePrevious) / deltaT;
    eIntegral = eIntegral + e * deltaT;

    // compute the PID control signal
    float u = (Kp * e) + (Kd * eDirivative) + (Ki * eIntegral);

    // update variables for the next iteration
    previousTime = currentTime;
    ePrevious = e;

    return u;
}

void serialPLotterMotor(int encoderCount) {
  tEncoder = millis();
if((tEncoder-previoustEncoder) > displaytEncoder){
  Serial.print(target);
  Serial.print("," );
  Serial.println(realTraget);
  Serial.print("," );
  Serial.println(encoderCount);
  previoustEncoder = tEncoder;
  }
}
