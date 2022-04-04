////////////////////////////////////////////////////////////////////////
/*
 * Programa desarrollado por Oscar Garcia, Sofía Esquivl y Cesar González  para el control cinmático
 * de robot omnidreccional implementado un sistema de visión estacionario.  
 * 
 * Medio de contacto
 * 
 * oscaremmanuel_garcia@ucol.mx
 * 
 * 
 */

////////////////////////////////////////////////////////////////////////




#include "PinChangeInterrupt.h"
#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/pidGain.h>

 
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>


#include <std_msgs/Int16.h>


ros::NodeHandle  nh;

#define LOOPTIME 10

void change_leftF_a();
void change_leftF_b();
void change_leftB_a();
void change_leftB_b();

void change_rightF_a();
void change_rightF_b();
void change_rightB_a();
void change_rightB_b();

double ComputePID(void);




/////////////////////////////////////////////////
unsigned long lastTime = 0,   sampleTime = 0;

double        Input1    = 0.0, Setpoint1   = 0.0;
double        ITerm1    = 0.0, lastInput1 = 0.0;


double        Input2    = 0.0, Setpoint2   = 0.0;
double        ITerm2    = 0.0, lastInput2 = 0.0;

double        Input3    = 0.0, Setpoint3   = 0.0;
double        ITerm3    = 0.0, lastInput3 = 0.0;

double        Input4    = 0.0, Setpoint4   = 0.0;
double        ITerm4    = 0.0, lastInput4 = 0.0;

double        outMin   = 0.0, outMax     = 0.0;
double        error1    = 0.0;
double        error2    = 0.0;
double        error3    = 0.0;
double        error4    = 0.0;


double        kpULF = 5.0  , kiULF = 0.1 ,   kdULF = 0.4;
double        kpULB = 5.0  , kiULB = 0.1  ,   kdULB = 0.4;
double        kpURF = 5.0  , kiURF = 0.1  ,   kdURF = 0.4;
double        kpURB = 6.0  , kiURB = 0.3  ,   kdURB = 2.0;


double        PTerm1    = 0.0, DTerm1 =  0.0,  lastError1 = 0.0;
double        PTerm2    = 0.0, DTerm2 =  0.0,  lastError2 = 0.0;
double        PTerm3    = 0.0, DTerm3 =  0.0,  lastError3 = 0.0;
double        PTerm4    = 0.0, DTerm4 =  0.0,  lastError4 = 0.0;
///////////////////////////////////////////////



double vU1 = 0.0, vU2 = 0.0, vU3 = 0.0, vU4 = 0.0;

const double valueConstLeftF = 2.23;
const double valueConstLeftB = 2.23;
const double valueConstRightF = 2.23;
const double valueConstRightB = 2.23;

double speedActFilterLF = 0.0, alphaLF = 0.3;
double speedActFilterLB = 0.0, alphaLB = 0.3;
double speedActFilterRF = 0.0, alphaRF = 0.3;
double speedActFilterRB = 0.0, alphaRB = 0.3;



Motor leftF(49, 51, 18, 19, 4); // Motor 1

Motor leftB(53, 47, 20, 21, 5);  // Motor 3

Motor rightF(50, 48, A10, A11, 3); // Motor 2

Motor rightB(46, 52, 10, 11, 2); //Motor  4


volatile long encoder1Pos = 0;    // encoder 1
volatile long encoder2Pos = 0;    // encoder 2
volatile long encoder3Pos = 0;    // encoder 3
volatile long encoder4Pos = 0;    // encoder 4


double demandx = 0;
double demandy = 0;
double demandz = 0;

double initPoseX = 0;
double initPoseY = 0;
double initPoseZ = 0;

float r = 0.05;  // Radio de llanta en m
float Lx = 0.192;   // Distancia de las llantas sobre el eje x en m
float Ly = 0.105;

float constR = 20; //   1/r


double demand_speed_leftF;
double demand_speed_leftB;
double demand_speed_rightF;
double demand_speed_rightB;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder1Diff;
float encoder2Diff;
float encoder3Diff;
float encoder4Diff;

float encoder1Error;
float encoder2Error;
float encoder3Error;
float encoder4Error;


float encoder1Prev;
float encoder2Prev;
float encoder3Prev;
float encoder4Prev;

int opcVisualization = 0;
int opcPower = 0;
int motorSintonizar = 0;

void cmd_vel_cb( const geometry_msgs::Twist& twist)   
{
  //Función para recibir las velocidades lineales y rotacional
    
  demandx = twist.linear.x;
  demandy = twist.linear.y;
  demandz = twist.angular.z;

  //Calcula el setpoint para cada una de las llantas 
  demand_speed_leftF  = constR * (demandx - demandy - Lx*demandz - Ly* demandz);
  demand_speed_rightF = constR * (demandx + demandy + Lx*demandz + Ly*demandz);
  demand_speed_leftB  = constR * (demandx + demandy - Lx*demandz - Ly*demandz);
  demand_speed_rightB = constR * (demandx - demandy + Lx*demandz + Ly*demandz);

  Setpoint1 = demand_speed_leftF;
  Setpoint2 = demand_speed_rightF;
  Setpoint3 = demand_speed_leftB;
  Setpoint4 = demand_speed_rightB;

}






void pidTunning (const custom_msgs::pidGain& pid_gain)
{
  motorSintonizar = pid_gain.motor;
  
  if (motorSintonizar == 1)
  {
    kpULF = pid_gain.kp;
    kiULF = pid_gain.ki;
    kdULF = pid_gain.kd;
  }
  else if (motorSintonizar == 3)
  {
    kpULB = pid_gain.kp;
    kiULB = pid_gain.ki;
    kdULB = pid_gain.kd;
  }

 else if (motorSintonizar == 2)
  {
    kpURF = pid_gain.kp;
    kiURF = pid_gain.ki;
    kdURF = pid_gain.kd;
  }
  else if (motorSintonizar == 4)
  {
    kpURB = pid_gain.kp;
    kiURB = pid_gain.ki;
    kdURB = pid_gain.kd;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );



ros::Subscriber<custom_msgs::pidGain> pid_gain_sub("pid_tunning", pidTunning);

geometry_msgs::Vector3Stamped speed_msg;   //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


double speed_act_leftF = 0;                    //Actual speed for left wheel in m/s
double speed_act_leftB = 0;                    //Actual speed for left wheel in m/s
double speed_act_rightF = 0;                    //Command speed for left wheel in m/s
double speed_act_rightB = 0;                    //Command speed for left wheel in m/s

const double encoder_cpr = 2816;               //Encoder ticks or counts per rotation


void setup()
{

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(pid_gain_sub);
  nh.advertise(speed_pub);                  
 
  
  //  Motor 1
  attachInterrupt(digitalPinToInterrupt(leftF.en_a), change_leftF_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftF.en_b), change_leftF_b, CHANGE);

  //Motor 3
  attachInterrupt(digitalPinToInterrupt(leftB.en_a), change_leftB_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftB.en_b), change_leftB_b, CHANGE);

  //Motor 2
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rightF.en_a), change_rightF_a, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rightF.en_b), change_rightF_b, CHANGE);

  //Motor 4
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rightB.en_a), change_rightB_a, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rightB.en_b), change_rightB_b, CHANGE);


  outMax =  100.0;                      // Límite máximo del controlador PID.
  outMin = -outMax;                     // Límite mínimo del controlador PID.

  sampleTime = 10;                      // Se le asigna el tiempo de muestreo en milisegundos.


}

void loop()
{
  ComputePID();
}




//Envia la velocidad en el marco de referncia del robot. En simulink las separa para obtener cada una de las llantas 
void publishSpeed(double time)
{
        speed_msg.vector.x = (r*(speedActFilterLF + speedActFilterLB + speedActFilterRF + speedActFilterRB))/4;    //Velocidad lineal
        speed_msg.vector.y = (r*(-speedActFilterLF + speedActFilterLB + speedActFilterRF - speedActFilterRB))/4;   //Velocidad lateral
        speed_msg.vector.z = (r*(-speedActFilterLF - speedActFilterLB + speedActFilterRF + speedActFilterRB))/(4*(Lx + Ly));;   //Velocidad rotacional 
        speed_pub.publish(&speed_msg);
  
}



// ************** Interrupciones de encoders **************
void change_leftF_a()
{
  // look for a low-to-high on channel A
  if (digitalRead(leftF.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftF.en_b) == LOW) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftF.en_b) == HIGH) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }

}

void change_leftF_b() {

  // look for a low-to-high on channel B
  if (digitalRead(leftF.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(leftF.en_a) == HIGH) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftF.en_a) == LOW) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }


}




// ************** encoder 3 *********************


void change_leftB_a()
{
  // look for a low-to-high on channel A
  if (digitalRead(leftB.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftB.en_b) == LOW) {
      encoder3Pos = encoder3Pos + 1;         // CW
    }
    else {
      encoder3Pos = encoder3Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftB.en_b) == HIGH) {
      encoder3Pos = encoder3Pos + 1;          // CW
    }
    else {
      encoder3Pos = encoder3Pos - 1;          // CCW
    }
  }

}

void change_leftB_b() {

  // look for a low-to-high on channel B
  if (digitalRead(leftB.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(leftB.en_a) == HIGH) {
      encoder3Pos = encoder3Pos + 1;         // CW
    }
    else {
      encoder3Pos = encoder3Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(leftB.en_a) == LOW) {
      encoder3Pos = encoder3Pos + 1;          // CW
    }
    else {
      encoder3Pos = encoder3Pos - 1;          // CCW
    }
  }


}

// ************** encoder 2 *********************

void change_rightF_a() {

  // look for a low-to-high on channel A
  if (digitalRead(rightF.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightF.en_b) == LOW) {
      encoder2Pos = encoder2Pos - 1;         // CW
    }
    else {
      encoder2Pos = encoder2Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightF.en_b) == HIGH) {
      encoder2Pos = encoder2Pos - 1;          // CW
    }
    else {
      encoder2Pos = encoder2Pos + 1;          // CCW
    }
  }

}

void change_rightF_b() {

  // look for a low-to-high on channel B
  if (digitalRead(rightF.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(rightF.en_a) == HIGH) {
      encoder2Pos = encoder2Pos - 1;         // CW
    }
    else {
      encoder2Pos = encoder2Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightF.en_a) == LOW) {
      encoder2Pos = encoder2Pos - 1;          // CW
    }
    else {
      encoder2Pos = encoder2Pos + 1;          // CCW
    }
  }


}



// ************** encoder 4 *********************

void change_rightB_a() {

  // look for a low-to-high on channel A
  if (digitalRead(rightB.en_a) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightB.en_b) == LOW) {
      encoder4Pos = encoder4Pos - 1;         // CW
    }
    else {
      encoder4Pos = encoder4Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightB.en_b) == HIGH) {
      encoder4Pos = encoder4Pos - 1;          // CW
    }
    else {
      encoder4Pos = encoder4Pos + 1;          // CCW
    }
  }

}

void change_rightB_b() {

  // look for a low-to-high on channel B
  if (digitalRead(rightB.en_b) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(rightB.en_a) == HIGH) {
      encoder4Pos = encoder4Pos - 1;         // CW
    }
    else {
      encoder4Pos = encoder4Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(rightB.en_a) == LOW) {
      encoder4Pos = encoder4Pos - 1;          // CW
    }
    else {
      encoder4Pos = encoder4Pos + 1;          // CCW
    }
  }


}



// Cálculo PID
double ComputePID(void)
  {
    if (millis() - lastTime >= sampleTime)   // LOOPTIME
      {
        noInterrupts(); // Se desactiva las intrrupciones por un instante 
        speed_act_leftF  = (valueConstLeftF * encoder1Pos) / (millis() - lastTime);
        speed_act_leftB  = (valueConstLeftB * encoder3Pos) / (millis() - lastTime);
        speed_act_rightF = (valueConstRightF * encoder2Pos) / (millis() - lastTime);
        speed_act_rightB = (valueConstRightB * encoder4Pos) / (millis() - lastTime);
  
        lastTime = millis();
        encoder1Pos = 0;
        encoder2Pos = 0;
        encoder3Pos = 0;
        encoder4Pos = 0;
  
        interrupts(); 

        speedActFilterLF = alphaLF*speed_act_leftF + (1.0-alphaLF)*speedActFilterLF;
        speedActFilterLB = alphaLB*speed_act_leftB + (1.0-alphaLB)*speedActFilterLB;
        speedActFilterRF = alphaRF*speed_act_rightF + (1.0-alphaRF)*speedActFilterRF;
        speedActFilterRB = alphaRB*speed_act_rightB + (1.0-alphaRB)*speedActFilterRB;
   
        Input1  = speedActFilterLF;
        Input3  = speedActFilterLB;
        Input2  = speedActFilterRF;
        Input4  = speedActFilterRB;
  
        error1  = (Setpoint1 - Input1)* kpULF;
        error3  = (Setpoint3 - Input3)* kpULB;
        error2  = (Setpoint2 - Input2)* kpURF;
        error4  = (Setpoint4 - Input4)* kpURB;
       
        ITerm1 += (error1 * kiULF);
        ITerm3 += (error3 * kiULB);
        ITerm2 += (error2 * kiURF);
        ITerm4 += (error4 * kiURB);
    
        DTerm1 = (error1 - lastError1) * kdULF;
        DTerm3 = (error3 - lastError3) * kdULB;
        DTerm2 = (error2 - lastError2) * kdURF;
        DTerm4 = (error4 - lastError4) * kdURB;

        lastError1 = error1;
        lastError2 = error2;
        lastError3 = error3;
        lastError4 = error4;

        
        if (ITerm1 > outMax) ITerm1 = outMax; else if (ITerm1 < outMin) ITerm1 = outMin;
        if (ITerm2 > outMax) ITerm2 = outMax; else if (ITerm2 < outMin) ITerm2 = outMin;//%%%%%%
        if (ITerm3 > outMax) ITerm3 = outMax; else if (ITerm3 < outMin) ITerm3 = outMin;//%%%%%%
        if (ITerm4 > outMax) ITerm4 = outMax; else if (ITerm4 < outMin) ITerm4 = outMin;//%%%%%%
    
        vU1 = error1 + ITerm1 + DTerm1;      
        vU2 = error2 + ITerm2 + DTerm2;       
        vU3 = error3 + ITerm3 + DTerm3;       
        vU4 = error4 + ITerm4 + DTerm4;       
    
        if (vU1 > outMax) vU1 = outMax; else if (vU1 < outMin) vU1 = outMin; // 
        if (vU2 > outMax) vU2 = outMax; else if (vU2 < outMin) vU2 = outMin; //
        if (vU3 > outMax) vU3 = outMax; else if (vU3 < outMin) vU3 = outMin; //
        if (vU4 > outMax) vU4 = outMax; else if (vU4 < outMin) vU4 = outMin; //
    
    
       
    
        rightB.rotate(vU4);
        leftF.rotate(vU1);
        leftB.rotate(vU3);
        rightF.rotate(vU2);
       
        publishSpeed(LOOPTIME);    
        nh.spinOnce();
      }
  }
