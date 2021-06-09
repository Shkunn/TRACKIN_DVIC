#include <Arduino.h>

String x_str;

String FR_received;
String RR_received;
String FL_received;
String RL_received;

String bf1;
String bf2;
String bf3;
String bf4;

int ind1;
int ind2;
int ind3;
int ind4;
int ind5;
int ind6;
int ind7;
int ind8;

int FR_fwd_speed;
int RR_fwd_speed;
int FL_fwd_speed;
int RL_fwd_speed;

#define SPEED 140    
#define TURN_SPEED 160    
#define speedPinR 9   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define RightMotorDirPin1  24    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  22   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  28    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  26   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B  6    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 5   //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 8    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B 7  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 12    //  Rear Wheel PWM pin connect Left MODEL-X ENB

/* Constantes pour les broches */
const byte TRIGGER_PIN = 36; // Broche TRIGGER
const byte ECHO_PIN = 38;    // Broche ECHO

const byte TRIGGER_PIN_B = 42; // Broche TRIGGER
const byte ECHO_PIN_B = 40;    // Broche ECHO

const byte TRIGGER_PIN_C = 46; // Broche TRIGGER
const byte ECHO_PIN_C = 44;    // Broche ECHO

const byte TRIGGER_PIN_D = 48; // Broche TRIGGER
const byte ECHO_PIN_D = 50;    // Broche ECHO
 
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_B = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_C = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_D = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;

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

    /* Initialise les broches */
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(ECHO_PIN, INPUT);

    pinMode(TRIGGER_PIN_B, OUTPUT);
    digitalWrite(TRIGGER_PIN_B, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(ECHO_PIN_B, INPUT);

    pinMode(TRIGGER_PIN_C, OUTPUT);
    digitalWrite(TRIGGER_PIN_C, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(ECHO_PIN_C, INPUT);

    pinMode(TRIGGER_PIN_D, OUTPUT);
    digitalWrite(TRIGGER_PIN_D, LOW); // La broche TRIGGER doit être à LOW au repos
    pinMode(ECHO_PIN_D, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(100); 

    //Serial.print("INIT BEGINS...");
    init_GPIO();

    go_advance(SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);
}

void loop() {

  if (Serial.available() > 0) 
  {    
    // read the incoming data:
    x_str             = Serial.readString();

    ind1              = x_str.indexOf('/');             //finds location of first ,
    bf1               = x_str.substring(0, ind1);       //captures first data String
    ind2              = x_str.indexOf('/', ind1+1 );
    FR_received       = x_str.substring(ind1+1, ind2);       //captures first data String
    ind3              = x_str.indexOf('/', ind2+1 );    //finds location of second ,
    bf2               = x_str.substring(ind2+1, ind3);
    ind4              = x_str.indexOf('/', ind3+1 );
    RR_received       = x_str.substring(ind3+1, ind4);  //captures second data String
    ind5              = x_str.indexOf('/', ind4+1 );
    bf3               = x_str.substring(ind4+1, ind5);
    ind6              = x_str.indexOf('/', ind5+1 );            //finds location of first ,
    FL_received       = x_str.substring(ind5+1, ind6);
    ind7              = x_str.indexOf('/', ind6+1 );    //finds location of second ,
    bf4               = x_str.substring(ind6+1, ind7);
    ind8              = x_str.indexOf('/', ind7+1 );
    RL_received       = x_str.substring(ind7+1, ind8);

    if(FR_received.toFloat() == 700 && RR_received.toFloat() == 700 && FL_received.toFloat() == 700 && RL_received.toFloat() == 700)
    {
        // Serial.print(" RIGHT SHIFT ");
        right_shift(200,200,200,200);
    }
    else if(FR_received.toFloat() == 600 && RR_received.toFloat() == 600 && FL_received.toFloat() == 600 && RL_received.toFloat() == 600)
    {
        // Serial.print(" LEFT SHIFT ");
        left_shift(200,200,200,200);
    }
    else if(FR_received.toFloat() == 0 && RR_received.toFloat() == 0 && FL_received.toFloat() == 0 && RL_received.toFloat() == 0)
    {
        stop_Stop();
    }

    // 0: Forward
    // 1: Backward

    if(bf1.toFloat() == 1)
    {
      // Serial.println("FR BCK");
      FR_bck(FR_received.toFloat());
    }
    else{
      // Serial.println("FR FWD");
      FR_fwd(FR_received.toFloat());
    }

    if(bf2.toFloat() == 1)
    {
      RR_bck(RR_received.toFloat());
    }
    else{
      RR_fwd(RR_received.toFloat());
    }

    if(bf3.toFloat() == 1)
    {
      FL_bck(FL_received.toFloat());
    }
    else{
      FL_fwd(FL_received.toFloat());
    }

    if(bf4.toFloat() == 1)
    {
      RL_bck(RL_received.toFloat());
    }
    else{
      RL_fwd(FR_received.toFloat());
    }
  }

  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);

  /* 1B. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_B, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_B, LOW);
  
  /* 2B. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_B = pulseIn(ECHO_PIN_B, HIGH, MEASURE_TIMEOUT_B);

  /* 1C. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_C, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_C, LOW);
  
  /* 2C. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_C = pulseIn(ECHO_PIN_C, HIGH, MEASURE_TIMEOUT_C);

  /* 1D. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_D, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_D, LOW);
  
//   /* 2D. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_D = pulseIn(ECHO_PIN_D, HIGH, MEASURE_TIMEOUT_D);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  float distance_mm_B = measure_B / 2.0 * SOUND_SPEED;
  float distance_mm_C = measure_C / 2.0 * SOUND_SPEED;
  float distance_mm_D = measure_D / 2.0 * SOUND_SPEED;

  String message = String(distance_mm)+"/"+String(distance_mm_B)+"/"+String(distance_mm_C)+"/"+String(distance_mm_D)+"/";
  // String message = String(0)+"/"+String(0)+"/"+String(0)+"/"+String(0)+"/";
  Serial.println(message);   
  Serial.flush();
}