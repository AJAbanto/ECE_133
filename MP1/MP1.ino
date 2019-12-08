#include <NewPing.h>

#define echoPin_F 32
#define trigPin_F 33
#define echoPin_L 31
#define trigPin_L 30
#define echoPin_R 34
#define trigPin_R 35
#define dirPin_L  9
#define stepPin_L 8
#define dirPin_R  11
#define stepPin_R 10
#define led_pin   13


#define MAX_DISTANCE 400

#define Wr 19   //robot width


int De = 0;
int Dw = 0;
int Dn = 0;



const int forward = 0;
const int backward = 1;


//long duration, cm;

NewPing sonar_F(trigPin_F, echoPin_F, MAX_DISTANCE);
NewPing sonar_R(trigPin_R, echoPin_R, MAX_DISTANCE);
NewPing sonar_L(trigPin_L, echoPin_L, MAX_DISTANCE);

void setup() {

    Serial.begin (9600);
    pinMode(led_pin , OUTPUT);
    pinMode(trigPin_F, OUTPUT);
    pinMode(echoPin_F, INPUT);
    pinMode(trigPin_L, OUTPUT);
    pinMode(echoPin_L, INPUT);
    pinMode(trigPin_R, OUTPUT);
    pinMode(echoPin_R, INPUT);

    pinMode(stepPin_R, OUTPUT);
    pinMode(dirPin_R, OUTPUT);
    pinMode(stepPin_L, OUTPUT);
    pinMode(dirPin_L, OUTPUT);

    unsigned int Dn_init = sonar_F.ping_cm();
    unsigned int De_init = sonar_R.ping_cm();
    unsigned int Dw_init = sonar_L.ping_cm();
    digitalWrite(led_pin,LOW);

    turn_180();
    auto_level();
    delay(2000);
}

void loop() {
  wall_follow();
  
 /*
  unsigned int Dn = sonar_F.ping_cm();
  unsigned int De = sonar_R.ping_cm();
  unsigned int Dw = sonar_L.ping_cm();
  
  Serial.print("Dn: ");
  Serial.print(Dn);
  Serial.print("cm");
  Serial.print(" De: ");
  Serial.print(De);
  Serial.print("cm");
  Serial.print(" Dw: ");
  Serial.print(Dw);
  Serial.print("cm");
  Serial.print("\n");
   */ 

}


void blink_1s_period(){
  digitalWrite(led_pin,HIGH);
  delay(1000);
  digitalWrite(led_pin,LOW);
  delay(1000);
}


//turn 180
void turn_180(){
  int i;
  for( i = 0; i<9450 ;i++){
      digitalWrite(dirPin_R, forward);
      digitalWrite(dirPin_L, backward);
  
      digitalWrite(stepPin_R, HIGH);
      digitalWrite(stepPin_L, HIGH);
      delayMicroseconds(40);
      digitalWrite(stepPin_R, LOW);
      digitalWrite(stepPin_L, LOW);
      delayMicroseconds(40);
  }
}

//turn 90 clock wise
void turn_90_cw(){
  int i;
  for( i = 0; i<4725 ;i++){
      digitalWrite(dirPin_R, forward);
      digitalWrite(dirPin_L, backward);
  
      digitalWrite(stepPin_R, HIGH);
      digitalWrite(stepPin_L, HIGH);
      delayMicroseconds(40);
      digitalWrite(stepPin_R, LOW);
      digitalWrite(stepPin_L, LOW);
      delayMicroseconds(40);
  }
}

//turn 90 counter clock wise
void turn_90_ccw(){
  int i;
  for( i = 0; i<4725 ;i++){
      digitalWrite(dirPin_R, backward);
      digitalWrite(dirPin_L, forward);
  
      digitalWrite(stepPin_R, HIGH);
      digitalWrite(stepPin_L, HIGH);
      delayMicroseconds(40);
      digitalWrite(stepPin_R, LOW);
      digitalWrite(stepPin_L, LOW);
      delayMicroseconds(40);
  }
}

void auto_level(){

  
  
  Dn = sonar_F.ping_cm();
  De = sonar_R.ping_cm();
  Dw = sonar_L.ping_cm();

  
  digitalWrite(dirPin_R, HIGH);
  digitalWrite(dirPin_L, LOW);

  //Auto level 
  while(Dw <= 10){
    
    for(int i = 0; i < 3 ; i++){
      analogWrite(stepPin_L,100);
      analogWrite(stepPin_R,100);
    }
  
    Dw = sonar_L.ping_cm();
  }

  //stop motors and stay in place
  analogWrite(stepPin_L,0);
  analogWrite(stepPin_R,0);
  
  
}



int kp = 20 , kd = 1 ,ki = 0;
void wall_follow(){
  
  int pid_val = 0;
  int err = 0 , last_err = 0;
  int wall_right = 0 , wall_left = 0;
  
  unsigned int Dn = 0 , De = 0 , Dw = 0;
  int base_delay = 50, pid_delay1 = 0, pid_delay2 = 0;
  unsigned int initial_dist = 0;
  unsigned int old_Dn = 0  , old_De = 0 , old_Dw = 0;
  
  Dn = sonar_F.ping_cm();
  De = sonar_R.ping_cm();
  Dw = sonar_L.ping_cm();

  initial_dist = Dn;

  //check existence of walls
  if(De < 40)
    wall_right = 1;

  if(Dw < 40)  
    wall_left  = 1;

  
  digitalWrite(dirPin_R, LOW);
  digitalWrite(dirPin_L, LOW);

  if(wall_right && wall_left){
    while(1){
  
      old_Dn = Dn;
      old_De = De;
      old_Dw = Dw;
      
      Dn = sonar_F.ping_cm();
      De = sonar_R.ping_cm();
      Dw = sonar_L.ping_cm();
          
     
  
      if(Dn <= (initial_dist - 25)){
        analogWrite(stepPin_L,0);
        analogWrite(stepPin_R,0);
        blink_1s_period();
        delay(2000);
        
        break;
      }
  
      last_err = err;

      //Choose which wall to follow based on availability
      if(wall_right && !wall_left)
        err =  De  - 10; 
      else if(wall_left && !wall_right)
        err = Dw - 10;
      else
        err = De - 10;
      
      pid_val =  kp * err + kd * ( err - last_err);
  
      pid_delay1 = constrain(base_delay + pid_val, 0 , 255);
      pid_delay2 = constrain(base_delay - pid_val, 0 , 255);
  
  
      for(int i = 0; i < 3 ; i++){
        analogWrite(stepPin_L,pid_delay1);
        analogWrite(stepPin_R,pid_delay2);
      }
  
    }
  }
}
