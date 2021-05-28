#include <Arduino.h>

String distance_received;
String position_received;
String x_str;

String FR_received;
String RR_received;
String FL_received;
String RL_received;

int x_int;
int ind1;
int ind2;
int ind3;
int ind4;

int FR_fwd_speed;
int RR_fwd_speed;
int FL_fwd_speed;
int RL_fwd_speed;

#define SPEED 140    
#define TURN_SPEED 160    
#define speedPinR 9   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  28   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 7    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B 8  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 12    //  Rear Wheel PWM pin connect Left MODEL-X ENB

/*motor control*/
void go_advance(int speed){
    RL_fwd(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_fwd(speed); 
}
void go_back(int speed){
    RL_bck(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_bck(speed); 
    }
void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck) {
    FL_fwd(speed_fl_fwd); 
    RL_bck(speed_rl_bck); 
    RR_fwd(speed_rr_fwd);
    FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd){
    FL_bck(speed_fl_bck);
    RL_fwd(speed_rl_fwd);
    RR_bck(speed_rr_bck);
    FR_fwd(speed_fr_fwd);
}

void left_turn(int speed){
    RL_bck(0);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(0); 
}
void right_turn(int speed){
    RL_fwd(speed);
    RR_bck(0);
    FR_bck(0);
    FL_fwd(speed); 
}
void left_back(int speed){
    RL_fwd(0);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(0); 
}
void right_back(int speed){
    RL_bck(speed);
    RR_fwd(0);
    FR_fwd(0);
    FL_bck(speed); 
}
void clockwise(int speed){
    RL_fwd(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(speed); 
}
void countclockwise(int speed){
    RL_bck(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(speed); 
}

void FR_fwd(int speed)  //front-right wheel forward turn
{
    digitalWrite(RightMotorDirPin1,HIGH);
    digitalWrite(RightMotorDirPin2,LOW); 
    analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
    digitalWrite(RightMotorDirPin1,LOW);
    digitalWrite(RightMotorDirPin2,HIGH); 
    analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
    digitalWrite(LeftMotorDirPin1,HIGH);
    digitalWrite(LeftMotorDirPin2,LOW);
    analogWrite(speedPinL,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
    digitalWrite(LeftMotorDirPin1,LOW);
    digitalWrite(LeftMotorDirPin2,HIGH);
    analogWrite(speedPinL,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
    digitalWrite(RightMotorDirPin1B, HIGH);
    digitalWrite(RightMotorDirPin2B,LOW); 
    analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
    digitalWrite(RightMotorDirPin1B, LOW);
    digitalWrite(RightMotorDirPin2B,HIGH); 
    analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
    digitalWrite(LeftMotorDirPin1B,HIGH);
    digitalWrite(LeftMotorDirPin2B,LOW);
    analogWrite(speedPinLB,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
    digitalWrite(LeftMotorDirPin1B,LOW);
    digitalWrite(LeftMotorDirPin2B,HIGH);
    analogWrite(speedPinLB,speed);
}
 
void stop_Stop()    //Stop
{
    analogWrite(speedPinLB,0);
    analogWrite(speedPinRB,0);
    analogWrite(speedPinL,0);
    analogWrite(speedPinR,0);
}

//Pins initialize
void init_GPIO()
{
    pinMode(RightMotorDirPin1, OUTPUT); 
    pinMode(RightMotorDirPin2, OUTPUT); 
    pinMode(speedPinL, OUTPUT);  

    pinMode(LeftMotorDirPin1, OUTPUT);
    pinMode(LeftMotorDirPin2, OUTPUT); 
    pinMode(speedPinR, OUTPUT);
    pinMode(RightMotorDirPin1B, OUTPUT); 
    pinMode(RightMotorDirPin2B, OUTPUT); 
    pinMode(speedPinLB, OUTPUT);  

    pinMode(LeftMotorDirPin1B, OUTPUT);
    pinMode(LeftMotorDirPin2B, OUTPUT); 
    pinMode(speedPinRB, OUTPUT);

    stop_Stop();
}

void setup()
{
    Serial.print("INIT BEGIN...");
    init_GPIO();

    go_advance(SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    go_back(SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    left_turn(TURN_SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    right_turn(TURN_SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(200,200,200,200); //right shift
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(200,200,200,200); //left shift
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(200,0,200,0); //left diagonal back
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(200,0,200,0); //right diagonal ahead
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(0,200,0,200); //left diagonal ahead
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(0,200,0,200); //right diagonal back
    delay(1000);
    stop_Stop();
    delay(1000);

    Serial.begin(115200);
    Serial.setTimeout(100);   

    Serial.print("INIT FINISHED...");
}

void loop() {

  if (Serial.available() > 0) 
  {
    // read the incoming data:
    x_str             = Serial.readString();

    ind1              = x_str.indexOf('/');  //finds location of first ,
    position_received = x_str.substring(0, ind1);
    distance_received = x_str.substring(ind1+1);

    ind1              = x_str.indexOf('/');  //finds location of first ,
    FR_received       = x_str.substring(0, ind1);   //captures first data String
    ind2              = x_str.indexOf('/', ind1+1 );   //finds location of second ,
    RR_received       = x_str.substring(ind1+1, ind2);   //captures second data String
    ind3              = x_str.indexOf('/', ind2+1 );
    FL_received       = x_str.substring(ind2+1, ind3);
    ind4              = x_str.indexOf('/', ind3+1 );
    RL_received       = x_str.substring(ind3+1);


    Serial.print("Received : ");
    Serial.print(FR_received);
    Serial.print(" / ");
    Serial.print(RR_received);
    Serial.print(" / ");
    Serial.print(FL_received);
    Serial.print(" / ");
    Serial.print(RL_received);


    if(FR_received.toFloat() == 500 && RR_received.toFloat() == 500 && FL_received.toFloat() == 500 && RL_received.toFloat() == 500)
    {
        Serial.print(" RIGHT SHIFT ");
        right_shift(200,200,200,200);
    }
    else if(FR_received.toFloat() == 600 && RR_received.toFloat() == 600 && FL_received.toFloat() == 600 && RL_received.toFloat() == 600)
    {
        Serial.print(" LEFT SHIFT ");
        left_shift(200,200,200,200);
    }
    else if(FR_received.toFloat() == 0 && RR_received.toFloat() == 0 && FL_received.toFloat() == 0 && RL_received.toFloat() == 0)
    {
        Serial.print(" STOP ");
        stop_Stop();
    }
    else
    {
      Serial.print(" MOTOR ACTION ");
      
      if(FR_received.toFloat() > 600)
      {
        FR_bck(FR_received.toFloat() - 600);
      }
      else{
        FR_fwd(FR_received.toFloat());
      }

      if(RR_received.toFloat() > 600)
      {
        RR_bck(RR_received.toFloat() - 600);
      }
      else{
        RR_fwd(RR_received.toFloat());
      }

      if(FL_received.toFloat() > 600)
      {
        FL_bck(FL_received.toFloat() - 600);
      }
      else{
        FL_fwd(FL_received.toFloat());
      }

      if(RL_received.toFloat() > 600)
      {
        RL_bck(RL_received.toFloat() - 600);
      }
      else{
        RL_fwd(FR_received.toFloat());
      }
    }
  }
}