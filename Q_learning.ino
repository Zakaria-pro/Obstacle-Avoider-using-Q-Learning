#include <Servo.h>
#include <SoftwareSerial.h> 

// Bluetooth_module
SoftwareSerial MyBlue(9, 8); // RX | TX 

// Servo motor
Servo myservo; // create servo object to control a servo
int pos = 0; // variable to store the servo position

// Ultrasonic Sensor 
#define trig A0
#define echo A1

// Motor B
#define right_backward 3
#define right_forward 2
#define right_speed 1

// Motor A
#define left_backward 4
#define left_forward 5
#define left_speed 6

#define SPEED 10
#define turn_speed 20

// for the Q learning algorithm
#define num_states 8
#define num_actions 4
#define iterations 100
#define gamma 0.3

int i, Qmax;
int current_state;
int action;
int next_state;

const int obstacles[num_states] = {0,2,1,4,3,6,5,7}; 

const int states[num_states] = {0,1,2,3,4,5,6,7}; //0: no obstacles //1:forward  //2:right //3:left //4:forward+right //5: forward +left //6: left +right //7: all

const int actions[num_actions] = {0,1,2,3}; //0: forward //1: backward //2:right //3: left

float dist;

/*
                                        {{38, 29, 28, 0},
                                         {21, 23, 22, 28},
                                         {0, 13, -1, 1},
                                         {14, 8, 1, 0},
                                         {21, 6, 0, 8},
                                         {0, 8, 0, -1},
                                         {13, 17, 5, 13},
                                         {0, 0, 0, 0}};
*/

/*
float Q_matrix[num_states][num_actions]={{41.00,18.00,20.00,29.00},
                                        
                                        {31.00,22.00,20.00,13.00},
                                        
                                        {24.00,20.00,17.00,17.00},
                                        
                                        {13.00,0.00,22.00,0.00},
                                        
                                        {18.00,20.00,15.00,24.00},  
                                        
                                        {0.00,19.00,17.00,22.00},
                                        
                                        {13.00,17.00,28.00,13.00},
                                        
                                        {22.00,0.00,0.00,19.00}};
*/

/*
float Q_matrix[num_states][num_actions]={{0,0,0,0},
                                        
                                        {0,0,0,0},
                                        
                                        {0,0,0,0},
                                        
                                        {0,0,0,0},
                                        
                                        {0,0,0,0},  
                                        
                                        {0,0,0,0},
                                        
                                        {0,0,0,0},
                                        
                                        {0,0,0,0}};
*/




// Matrix
float Q_matrix[num_states][num_actions]={{1       , -1           , 0        , 0 }, // 0: no obstacles        
                                        
                                        {-1       , 1            , 0        , 0 }, // 1:forward obstacle
                                        
                                        {0        , -1            , -1       , 1 }, // 2:right obstacle
                                        
                                        {0        , 0            , 1        , -1}, // 3:left onstacle
                                        
                                        {-1       , 0            , -1       , 1 }, // 4:forward + right obstacles
                                        
                                        {-1       , 0            , 1        , -1}, // 5: forward + left obstacles
                                        
                                        {0        , 1            , -1       , -1}, // 6: left + right obstacles
                                        
                                        {-1       , 1            , -1       , -1}};
                                         
                                         


                                      //0: forward //1: backward //2:right //3: left
float rewards[num_states][num_actions]={{1       , -1           , 0        , 0 }, // 0: no obstacles        
                                        
                                        {-1       , 1            , 0        , 0 }, // 1:forward obstacle
                                        
                                        {0        , -1            , -1       , 1 }, // 2:right obstacle
                                        
                                        {0        , 0            , 1        , -1}, // 3:left onstacle
                                        
                                        {-1       , 0            , -1       , 1 }, // 4:forward + right obstacles
                                        
                                        {-1       , 0            , 1        , -1}, // 5: forward + left obstacles
                                        
                                        {0        , 1            , -1       , -1}, // 6: left + right obstacles
                                        
                                        {-1       , 1            , -1       , -1}};// 7: left + right + forward obstacle



// Usefull Functions
float Distance();
void Stop();
void GoForward();
void GoBackward();
void GoRight();
void GoLeft();
int Get_state();
int NextAction(int state);
void ActionExecution(int action);

int tim=0;

void setup() {
  Serial.begin(9600);
  pinMode(right_speed,OUTPUT);
  pinMode(left_speed,OUTPUT);
  pinMode(right_backward,OUTPUT);
  pinMode(right_forward,OUTPUT);
  pinMode(left_forward,OUTPUT);
  pinMode(left_backward,OUTPUT);
  
  myservo.attach(13);
  
  pinMode(trig, OUTPUT);
  digitalWrite(echo, LOW);

  MyBlue.begin(9600);  
  MyBlue.println("Ready to connect"); 
}


void loop() {
  
  //training
  MyBlue.println("training begin");
  current_state = GetState();
  for( tim = 0; tim < iterations; tim++){
    MyBlue.println("---------------------------------");
    MyBlue.print("Iteration :"); 
    MyBlue.print(tim);
    MyBlue.print("\n");
    action = NextAction(current_state);
    //execute action
    ActionExecution(action);
    //update Q_matrix
    next_state = GetState();
    Qmax = -100;
    for ( i = 0; i < num_actions ; i++) {
      if (Qmax < Q_matrix[next_state][i])
        Qmax = Q_matrix[next_state][i];
    } 
    Qmax *= gamma;
    Q_matrix[current_state][action] = rewards[current_state][action] + Qmax;
    current_state = next_state;
  }
  MyBlue.println("training ends. Q_Matrix:");
  
  
  //printing the Q matrix 
  for (i =0; i< num_states ; i++){
    MyBlue.print("{");
    for( int j=0; j< num_actions; j++){
      MyBlue.print(Q_matrix[i][j]);
      if(j == num_actions-1){
        continue;
      }
      MyBlue.print(",");
    }
    MyBlue.print("},");
    MyBlue.println("\n");
  }
  
  
  
  // Validation
  MyBlue.println("---------------------------------");
  MyBlue.println("validation begin");
  MyBlue.println("---------------------------------");
  while(1){
    current_state = GetState();
    Qmax = -100;
    for (i=0; i< num_actions; i++){
      if (Qmax < Q_matrix[current_state][i]){
        Qmax = Q_matrix[current_state][i];
        action = i;
      }
    }
    ActionExecution(action);
    MyBlue.println("---------------------------------");
  }

}
  

int GetState(){
  int obstacle = 0;
  pos = 0; 
  Stop();
  delay(1000);
  
  // Sensor looking right
  myservo.write(30);
  MyBlue.println("Looking Right");
  delay(1500);
  dist = Distance();
  if (dist <30 && dist>2) obstacle += 1;
  
  // Sensor looking forward 
  myservo.write(90);
  MyBlue.println("Looking Forward");
  delay(1500);
  dist = Distance();
  if (dist <30 && dist>2) obstacle += 2;
  
  // Sensor looking left 
  myservo.write(160); 
  MyBlue.println("Looking Left");
  delay(1500);
  dist = Distance();
  if (dist <30 && dist>2) obstacle += 4;
  
  
  //Knowing your state
  for (int i=0; i<num_states; i++){
     if(obstacle == obstacles[i]) {
        MyBlue.print("returning state number :");
        MyBlue.print(i); 
        return i;
     }
  }
  
}

// take random actions in training
int NextAction(int state){
  int index = rand()%4;
  return index;
}



#define p 50000
#define c 25000

void ActionExecution(int action){
  int k;
  myservo.write(90);
  delay(1000);
  if (action == 0){
      //forward 
      MyBlue.println("going forward");
      GoForward();
      
      for(k=0; k<p;k++){
        delay(100);
        dist = Distance();
        if (dist < 30 && dist > 2){
          Stop();
          MyBlue.println("finding an obstacle ");
          break;}
      }
      
  }else if (action == 1){
      //backward
      MyBlue.println("going backwards");
      GoBackward();
      delay(1000);
      GoLeft();
      
  }else if(action == 2){
      //go right
      MyBlue.println("going right");
      GoBackward();
      delay(300);
      GoRight();
      GoForward();
      for(k=0; k<p;k++){
        delay(200);
        dist = Distance();
        if (dist < 30 && dist > 2){
          Stop();
          MyBlue.println("finding an obstacle ");
          break;}
      }
      
  }else if (action == 3){
      MyBlue.println("going Left");
      //go left
      GoBackward();
      delay(300);
      GoLeft();
      GoForward();
      for(k=0; k<p;k++){
        delay(200);
        dist = Distance();
        if (dist < 30 && dist > 2){
          Stop();
          MyBlue.println("finding an obstacle ");
          break;}
      }
  }
  delay(1000);
}


float Distance(){
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  int duree = pulseIn(echo,HIGH);
  float dist = 0.034*duree/2;
  MyBlue.print("La distance est :");
  MyBlue.print(dist);
  MyBlue.print("\n");
  return dist;
}

void Stop(){
  digitalWrite(right_backward,LOW);
  digitalWrite(left_backward,LOW);
  digitalWrite(right_forward,LOW);
  digitalWrite(left_forward,LOW);
}
void GoForward(){
  analogWrite(right_speed,10);
  analogWrite(left_speed,10);
  digitalWrite(right_backward,LOW);
  digitalWrite(left_backward,LOW);
  
  digitalWrite(right_forward,HIGH);
  digitalWrite(left_forward,HIGH);
}
void GoBackward(){
  digitalWrite(right_forward,LOW);
  digitalWrite(left_forward,LOW);
  //analogWrite(right_speed,SPEED);
  //analogWrite(left_speed,SPEED);
  digitalWrite(right_backward,HIGH);
  digitalWrite(left_backward,HIGH);
}
void GoRight(){
  digitalWrite(right_forward,LOW);
  digitalWrite(right_backward,LOW);
  //analogWrite(right_speed,SPEED);
  analogWrite(left_speed, turn_speed);
  digitalWrite(left_forward,HIGH);
  digitalWrite(left_backward,LOW);
  delay(700);
}
void GoLeft(){
  digitalWrite(left_backward,LOW);
  digitalWrite(left_forward,LOW);
  analogWrite(right_speed,turn_speed);
  //analogWrite(left_speed,SPEED);
  digitalWrite(right_backward,LOW);
  digitalWrite(right_forward,HIGH);
  delay(700);
}
