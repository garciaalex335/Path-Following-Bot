/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "servo.h"
#include "mstimer.h"

 const int center = 3000; // position for center of IR sensor


// initialize global to store qtr values
static volatile int sensorValuesCalibrated[] = {0,0,0,0,0,0,0,0};

// initialize array to store out path and the corresponding steps
static volatile int path[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static volatile int steps[25]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};


//initialize drivemode. 0 for straight, 1 or 2 for turns, -1 after task completion to stop
// updated by qtr at intersection detection and referenced by cog which controls servos
static volatile int driveMode = 0;

// position updated by cog running qtr and referenced by cog running servos to correct position
static volatile int position = 3000;


// flags used for indicators. Updated by qtr and ping cogs. Referenced by indicator cog.
static volatile int object = 0;
static volatile int atIntersect = 0;
static volatile int obstacle = 0;
static volatile int obstacleFound = 0;
static volatile int stepsToObstacle = 0;

static volatile int enemy = 0;
static volatile int friend = 0;

static volatile int atObject = 0;
static volatile int checkSignal = 0;

// pins for servos
const int rightServo = 17;
const int leftServo = 16;
const int armServo = 15;


// 1 and 3 are moving up and down along "avenues" 2 and 4 are moving across "streets"
// updated by cog controlling servos, referenced by cog controlling pings 
// to help determine if detected object is obstacle or desired object
static volatile int direction = 1; 
 
// speeds used in turns 
int fastTurn = 25; 
int slowTurn = 18;
 
// memory allocation for cogs 
unsigned int stack1[40+25];
unsigned int stack2[40+25];
unsigned int stack3[40+25];
unsigned int stack4[40+25];
unsigned int stack5[40+25];

// variable to track 
static volatile int stepIndex = 0;

static volatile int finishedPath = 0;

static volatile int activeArm = 0;


////// function prototypes ///////
void readLine(void *par);

void followLine(void *par);

void pings(void);

void indicate(void *par);


void turnLeftAlt(void);

void turnRightAlt(void);

int getPath(int adjM[14][14], int str, int end, int pathShort[10]);

int get2Path(int adjM[14][14], int str, int L1, int L2);

void getSteps(int numSteps);

void Dijkstra(int);

void updateDirection(int);

void operateArm(void *par);

void objectWatch(void *par);

void turn180(void);

void turn(void);
 
////////////////////////////////////




int main()                                    // Main function
{
  
  // start timer
  mstime_start();
  
  
  pause(300); // this was included during testing when using terminal. Could likely be removed
  
  
  
//  int num = Dijkstra(1);
  
//  for(int i = 0; i < num; i++){
 //   printf("Dij = %d\n",steps[i]);
 // }


  
  
  
  

  

  cogstart(&readLine,NULL,stack1,sizeof(stack1)); // cog running qtr and updating position
  cogstart(&followLine,NULL,stack2,sizeof(stack2)); // cog running servos
  cogstart(&objectWatch,NULL,stack3,sizeof(stack3)); // cog running ultrasonics
  cogstart(&indicate,NULL,stack4,sizeof(stack4)); // cog running indicators
  cogstart(&operateArm,NULL,stack5,sizeof(stack5));// cog to operate the arm
 
 
  while(1)
  {
      pings();
  //  for(int i = 0; i < 25; i++){
  //   printf("steps = %d \n",steps[i]); 
  //  }      
  //  printf("\n");
    
  //  pause(500);
    
    //printf("mode = %d \t steps = %d\n",driveMode,stepsToObstacle);
    //printf("D = %d \t mode = %d\n",finishedPath,driveMode);
  
          



  }  
}


void readLine(void *par)
{
        // values used for calibration of qtr data
           int minV =180;
           int maxV = 3500;
           int de = maxV-minV;

// flag to mark passing "start" 
    static int reachedI0 = 0;

   // how many sensors on the IR array
  const uint8_t SensorCount = 8;  


// initialize sensor values and pins
  int sensorValues[] = {0,0,0,0,0,0,0,0};
  int _sensorPins[] =  {1,2,3,4,5,6,7,8};
  
  
   while(1){
    
    pause(175); // pause which was required to get appropriate values during testing
    
    
      for (uint8_t i = 0; i < SensorCount; i ++)
      {
        sensorValues[i] = maxV;
        
      }
      
      // make sensor line an output (drives low briefly, but doesn't matter)
      // drive sensor line high
      set_directions(8,1,255);
      set_outputs(8,1,255);

      pause(1); // charge lines for 1000 us
      
      set_direction(1,0); // make all inputs

      // record time to signals going low    
             
      sensorValues[0]=rc_time(1,1);
      set_direction(2,0);
      sensorValues[1]=rc_time(2,1);
      set_direction(3,0);
      sensorValues[2]=rc_time(3,1);
      set_direction(4,0);
      sensorValues[3]=rc_time(4,1);
      set_direction(5,0);
      sensorValues[4]=rc_time(5,1);
      set_direction(6,0);
      sensorValues[5]=rc_time(6,1);
      set_direction(7,0);
      sensorValues[6]=rc_time(7,1);
      set_direction(8,0);
      sensorValues[7]=rc_time(8,1);


      // normalize to 0-1000           
      for(int i = 0; i < SensorCount;i++){
        //printf("values = %d\n",sensorValues[i]);
          sensorValuesCalibrated[i] =  ((sensorValues[i]-minV)*1000)/de;
          
          if(sensorValuesCalibrated[i] > 1000){
           sensorValuesCalibrated[i] = 1000; 
          }
          
          if(sensorValuesCalibrated[i] < 0){
           sensorValuesCalibrated[i] = 0; 
          }    
        //  printf("values = %d\n",sensorValuesCalibrated[i]);
                         
      }             
          // printf("\n");
      // calculate position    
        int average = 0;
        int sum = 0;
        int value = 0;
        int count = 0;
        static int lastLineValue = 0;
        int online = 0;
        
        for(int j = 0; j < SensorCount; j++){
          value = sensorValuesCalibrated[j];
          
          if(value > 200){ // set a flag that at least one sensor saw the line
           online = 1; 
          }            
          
          if(value > 100){
           average += value*(j*1000); 
           sum += value;
          }     
          
          if(value > 600){
           count++; // count how many sensors definitely saw the line
            
          }                   
          
        }  
        
        // if we've gone off the line
        if(online == 0){
         if(lastLineValue < 3500){
          lastLineValue = 0; 
         }
         else{
          lastLineValue = 7000; 
         }                      
        }                  
        else{
          lastLineValue = average/sum;
        }        
        
        // if all the sensors indicate an intersection
        if(count == SensorCount && atIntersect == 0){
          
          while(atObject){}
          
          
          atIntersect = 1; // set the flag for the indicator 
          
          if(obstacleFound == 0){stepsToObstacle++;}
          
          if(reachedI0 == 1 && obstacleFound == 1){ // if we havent already detected the intersection
           
           driveMode = steps[stepIndex]; // determine the next step
           
          
           stepIndex++; // update the index of our path array
          }            
           // set global flag that we are at the intersection
          reachedI0 = 1;
        }
        else if(count < SensorCount){
          atIntersect = 0; // clear the flag for indication
        }                    

        
      // update the global variable for position
      position = lastLineValue;
      //printf("pos = %d\n",position);

 }
}


void followLine(void *par) // posi indicates line position on IR sensor, driveMode is either Stop (0), Forward (1)
{
  
  


const int iniMotorPower = 30; // baseline speed

const int innerBand = 400; // +- from center to make small heading adjustments
const int outterBand = 1000; // +- from center to make big heading adjustments

while(1){
  printf("enemy = %d \t friend = %d \n",enemy,friend);





  
  // do we need to turn?
  if(driveMode == 1){
   turnRightAlt(); 
  }    
  else if(driveMode == 2){
   turnLeftAlt(); 
  }
  else if(driveMode == 3){
    turn180(); 
  }
  else if(driveMode == 4){
    //printf("hi");
   servo_speed(leftServo,0); 
   servo_speed(rightServo,0);
   
   pause(200);      
    turn();
  }
  else if(driveMode == 5){
    servo_speed(leftServo,40);
    servo_speed(rightServo,-40);
    
    pause(1250);
    
   servo_speed(leftServo,0); 
   servo_speed(rightServo,0);
   
   pause(2000);  
   
   while(atObject){}
   
    servo_speed(leftServo,-40);
    servo_speed(rightServo,40);
    
    pause(500); 
    turnRightAlt();
  }    
  else if(driveMode == -2){
   servo_speed(leftServo,15); 
   servo_speed(rightServo,-15);
   //pause(2 
  }        
  else if(driveMode == -1){ // have we completed all necessary steps?
    servo_speed(leftServo,40);
    servo_speed(rightServo,-40);
  
    pause(1000);  
    
   servo_speed(leftServo,0); 
   servo_speed(rightServo,0); 
   break; // were done!
  }
  else if(driveMode == -4){
   servo_speed(leftServo,0); 
   servo_speed(rightServo,0); 
  }            
  
  
  
  // how much to adjust speed by
 static int adjustHeading = 0;

  // variables for timing a heading adjustment
 int steppingTimeNew;
 static int steppingTimeOld = 0;

 // heading can be adjusted every 750 ms. Only run at adjusted heading for 250ms before recentering.
 int stepDelay = 350;
 int unStepDelay = 150;


 steppingTimeNew = mstime_get();
 if((steppingTimeNew - steppingTimeOld > stepDelay) ){ // if its time to adjust the heading

  steppingTimeOld = mstime_get();

  // check the position relative to inner bands and make a small adjustment
  if(position < center - innerBand){
    adjustHeading = 12; 

    // compare against the outter band if a bigger adjustment is necessary
     if(position < center - outterBand){
      adjustHeading = 35; 
     }
  } 
  else if(position > center + innerBand){// do we need to adjust the other way
    adjustHeading = -12;

     if(position > center + outterBand){
      adjustHeading = -35; 
     }
  }
 }


// reset heading
 if(steppingTimeNew - steppingTimeOld > unStepDelay){

  adjustHeading = 0;
 }


//  adjustHeading = 0;



  // calculate the speeds for each motor
  int leftMotorSpeed =  iniMotorPower + adjustHeading;
  int rightMotorSpeed = - iniMotorPower + adjustHeading;


  



  // write to the servos
  if(driveMode != -2 && driveMode != -4){
    servo_speed(leftServo,leftMotorSpeed);
    servo_speed(rightServo,rightMotorSpeed);
  }
  
 }
}





int ping(int pinTrig, int pinEcho){
 low(pinTrig); // write the trigger low
 pulse_out(pinTrig,10); // write the trigger high 
 return pulse_in(pinEcho,1); // record time of flight on echo pin
  
}  



void pings(void){                                  

  // pin assignments for USS

  int pinTrigF = 10;
  int pinEchoF = 11;
  
  int pinTrigL = 12;
  int pinEchoL = 13;
  
//  int pinTrigR = 10;
//  int pinEchoR = 11;
  
  
  
  long tEcho = 0;
  
  
  // vars for distances
  int fDist = 0;
  int rDist = 0;
  int lDist = 0;
 
  while(1)
  {
   //// ping each sensor and record distances in cm 
    tEcho = ping(pinTrigF,pinEchoF);
    fDist = tEcho/58;
    
//    tEcho = ping(pinTrigR,pinEchoR);
//    rDist = tEcho/58;
    
    tEcho = ping(pinTrigL,pinEchoL);
    lDist = tEcho/58;    

   // printf("Dist = %d\n",lDist);

    if(fDist < 5 && obstacleFound == 0 && driveMode == 0){
     obstacle = 1; // indicate obstacle
     obstacleFound = 1; // we found the obstacle
     
 //    int startVertex = stepsToObstacle - 2; // where do we start our pathing from


  
     // obstacle to avoid
     int obstaclePos =stepsToObstacle - 1; // legal values are 1,2,4
     // steps array creation
    // printf("ob = %d \n",obstaclePos);
     Dijkstra(obstaclePos);
     
     driveMode = 4; // turn around
     
     
   //  for(int i = 0; i < 25; i++){
   //   printf("steps = %d \n",steps[i]); 
   //  }      
   //  printf("\n");
   //  printf("mode = %d\n",driveMode);
     
     

    }
    else if(obstacleFound && fDist < 5 && steps[stepIndex]==4 && !obstacle && !atObject){
      obstacle = 1;
      driveMode = steps[stepIndex]; // determine the next step
           
          
      stepIndex++; // update the index of our path array

    } 
    else if(fDist < 5 && !atObject){
      obstacle = 1;    
    }      
    else {
     obstacle = 0; 
    }           
    
    /*
    
    Key for converting map intersection to graph nodes
    
    []    [8]-->[7]-->[6]-->[5]   [A5]    [A4]--->[A3]--->[A2]--->[A1]
           ↑     ↓     ↑     ↓             ↑       ↓       ↑       ↓
    [4]---[3]---[2]---[1]---[0]   [I5]----[I4]----[I3]----[I2]----[I1]
     ↓     ↑     ↓     ↑     ↓     ↓       ↑       ↓       ↑       ↓
    [13]<-[12]<-[11]<-[10]<-[9]   [B5]<---[B4]<---[B3]<---[B2]<---[B1]
              
     
    
     */

    
     

//    if(direction%2 ==0 && (rDist < 30 || lDist < 30)){ // are we moving across "streets" and there is something to the side?
//      obstacle = 1;
//    }
//    else if(direction %2 == 1 && (fDist < 30 || rDist < 30)){ // are we moving up "avenues" and there is something in front or to the right?
//      obstacle = 1;
//    }
//    else{
//     obstacle = 0; 
//    }   
            
    
    if(lDist < 10 && (driveMode == 0 || driveMode == 5 || driveMode == -1) && !activeArm){
     //atObject = 1; 
     object = 1; 
     
    }
    else if(driveMode == -2|| driveMode == -3){
      object = 1;
    }              
    else{
     object = 0; 
    }      
    
    

  //if(direction%2 == 1 && lDist < 10){ // are we moving up "avenues" and there is something to the left?   
  //   object = 1;       
  //  }
  //  else if(direction %2 == 0 && fDist < 10){   // are we moving across "streets" and there is something in front?
  //   object = 1;    
  //  }
  //  else{
  //   object = 0; 
  //  }         
      
    

  }  
   
  
}



void indicate(void *par){
  
  // pin assignments
  int intersectLED = 27;
  int friendLED = 9;
  //int obstacleLED = 9;
  int enemyLED = 26;
  
  // signals from the rPi
  int enemySig = 14;
  int friendSig = 0;
  
  
  // set LEDs to output
  set_direction(intersectLED,1);
  set_direction(friendLED,1);
  //set_direction(obstacleSig,1);
  set_direction(enemyLED,1);
  
  // set signal from rPi to input
  set_direction(enemySig,0);
  set_direction(friendSig,0);
  
  int friendCheck;
  int enemyCheck;
  
  
  // set output based on the flags set by qtr and ping cogs
  while(1){
   
   friendCheck = 0;
   enemyCheck = 0;
   //printf("\n");
   if(checkSignal){
    friendCheck = input(friendSig);
    enemyCheck = input(enemySig);
    //printf("friend = %d \t enemy = %d\n",friendCheck,enemyCheck);

   }   
   
   friend = object & friendCheck;
   enemy = object & enemyCheck;
   


    pause(100);
    
    
    

   //set_output(obstacleSig,obstacle);
   
  // printf("ob = %d\n",friendCheck);
   
   if(obstacle){
     set_output(intersectLED,1);
     set_output(friendLED, 1);
     set_output(enemyLED, 1);  
     pause(300);    
     
   }
   else if(enemy){
    set_output(intersectLED,0);
    set_output(friendLED, 0);
    set_output(enemyLED, 1); 
    pause(500);    
   } 
   else if(friend){
       set_output(intersectLED,0);
     set_output(friendLED, 1);
     set_output(enemyLED, 0);
     pause(500);   
   }
   else if(atIntersect){
       set_output(intersectLED,1);
    set_output(friendLED, 0);
     set_output(enemyLED, 0); 
     pause(200); 
   }  
   else{
       set_output(intersectLED,0);
    set_output(friendLED, 0);
    set_output(enemyLED, 0); 
   }    
   
   
             
  
  }  
  
  
}  


void turnRightAlt(){
    //turn right
    
    // roll forward to clear intersection
  servo_speed(leftServo,40);
  servo_speed(rightServo,-40);
  
  pause(500);  
    
    
    // begin turning
  if(driveMode !=5){servo_speed(leftServo,fastTurn);}
  else{servo_speed(leftServo,1.5*fastTurn);}
  servo_speed(rightServo,fastTurn);

  pause(250);

  // continue turning until the edge of the sensor detects the line to the right
  while(sensorValuesCalibrated[0] < 600){

  }

  // slow down
  servo_speed(leftServo,slowTurn);
  servo_speed(rightServo,slowTurn);


  // continue turning until the next sensor detects the line
  while(sensorValuesCalibrated[2] < 600){

  }
  servo_speed(leftServo,0);
  servo_speed(rightServo,0);
  
  // turn complete
  driveMode = 0;
  pause(100);
  updateDirection(-1);
}

void turnLeftAlt(){
  // roll forward to clear intersection
  servo_speed(leftServo,40);
  servo_speed(rightServo,-40);
  
  pause(500);  
    
  
    //turn left
  servo_speed(leftServo,-fastTurn);
  servo_speed(rightServo,-fastTurn);



  //  turn until the edge of the sensor detects the line to the left
  while(sensorValuesCalibrated[7] < 600){

  }

  // slow down
  servo_speed(leftServo,-slowTurn);
  servo_speed(rightServo,-slowTurn);



// continue turning until the next sensor detects the line
  while(sensorValuesCalibrated[5] < 600){

  }

  
  //turn complete
  driveMode = 0;
  updateDirection(1);
}


void turn180(){
    //turn right
    
    // roll forward to clear intersection
  servo_speed(leftServo,40);
  servo_speed(rightServo,-40);
  
  pause(500);  
    
    
    // begin turning
  servo_speed(leftServo,fastTurn);
  servo_speed(rightServo,fastTurn);

  // continue turning until the edge of the sensor detects the line to the right
  while(sensorValuesCalibrated[0] < 600){

  }

  // slow down
  servo_speed(leftServo,slowTurn);
  servo_speed(rightServo,slowTurn);


  // continue turning until the next sensor detects the line
  while(sensorValuesCalibrated[1] < 600){

  }
  
  while(sensorValuesCalibrated[1] > 300){
    
  }    

  servo_speed(leftServo,-40);
  servo_speed(rightServo,40);
  
  pause(200);  
    
  servo_speed(leftServo,0);
  servo_speed(rightServo,0); 
  // turn complete
  driveMode = 1;
  updateDirection(-1);
}

void turn(){
    //turn right
 // printf("start");  
  servo_speed(leftServo,-40);
  servo_speed(rightServo,40);
  
  pause(500);  
    
    // begin turning
  servo_speed(leftServo,1.25*fastTurn);
  servo_speed(rightServo,fastTurn);

  // continue turning until the edge of the sensor detects the line to the right
  while(sensorValuesCalibrated[0] < 600){

  }

  // slow down
  servo_speed(leftServo,slowTurn);
  servo_speed(rightServo,slowTurn);


  // continue turning until the next sensor detects the line
  while(sensorValuesCalibrated[5] < 600){

  }
  
  servo_speed(leftServo,-40);
  servo_speed(rightServo,40);
  
  pause(500);  
  /*
  servo_speed(leftServo,slowTurn);
  servo_speed(rightServo,slowTurn);


  // continue turning until the next sensor detects the line
  while(sensorValuesCalibrated[7] < 600){

  }
  */
  
  // turn complete
  driveMode = 0;
  updateDirection(2);
//  printf("completed turn");
}

void updateDirection(int turn){
  
// 1 and 3 are moving up and down along "avenues" 2 and 4 are moving across "streets"
// updated by cog controlling servos, referenced by cog controlling pings 
// to help determine if detected object is obstacle or desired object
  
  
 direction += turn; // update the global direction variable
 
 if(direction < 1){ // roll over if below 1
   direction += 4;
 }    
 
 if(direction > 4){ // roll over if above 4
  direction -=4; 
 }   
  
}  


void Dijkstra(int obs)
{
    
    int path1[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int path2[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int path3[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int path4[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    int numSteps;
    int numSteps1;
    int numSteps2;
    int numSteps3;
    int numSteps4;
    int start = obs-1;
    
    int id[14]={0,1,2,3,4,5,6,7,8,9,10,11,12,13}; // all possible nodes
    
    // array which stores all initial adjacencies, i.e. what are our options at each intersection based on traffic laws?
    int adj[14][14] = {{0,1,0,0,0,0,0,0,0,1,0,0,0,0},
                       {1,0,1,0,0,0,1,0,0,0,0,0,0,0},
                       {0,1,0,1,0,0,0,0,0,0,0,1,0,0},
                       {0,0,1,0,1,0,0,0,1,0,0,0,0,0},
                       {0,0,0,1,0,0,0,0,0,0,0,0,0,1},
                       {1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                       {0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                       {0,0,1,0,0,0,1,0,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                       {0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                       {0,1,0,0,0,0,0,0,0,0,0,1,0,0},
                       {0,0,0,0,0,0,0,0,0,0,0,0,1,0},
                       {0,0,0,1,0,0,0,0,0,0,0,0,0,1},
                       {0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
                       
     // update the adjacencies for wherever the obstacle is to be none                  
    for(int i = 0;i<14;i++)
    {
        if(i!=obs+1){
            adj[i][obs]=0;
            adj[obs][i]=0;
        }
    }
    
    // call method which runs Dijkstra's multiple times for each possible combination from start to each destination
    if(obs!=4){
        numSteps1 = getPath(adj,start,obs,path1);
        numSteps2 = getPath(adj,obs,4,path2);
        numSteps3 = getPath(adj,4,5,path3);
        numSteps4 = getPath(adj,5,13,path4);
        numSteps = numSteps1+numSteps2+numSteps3+numSteps4;
    }
    else
    {
        numSteps1 = getPath(adj,start,9,path1);
        numSteps2 = getPath(adj,9,8,path2);
        numSteps3 = getPath(adj,8,5,path3);
        numSteps4 = getPath(adj,5,13,path4);
        numSteps = numSteps1+numSteps2+numSteps3+numSteps4;   
    }
    
        for(int i = 0;i<numSteps1;i++)
    {
        path[i]=path1[i];
    }
    for(int i = 0;i<numSteps2;i++)
    {
        path[i+numSteps1]=path2[i];
    }
    for(int i = 0;i<numSteps3;i++)
    {
        path[i+numSteps1+numSteps2]=path3[i];
    }
    for(int i = 0;i<numSteps4;i++)
    {
        path[i+numSteps1+numSteps2+numSteps3]=path4[i];
    }
    // update the array of steps to be a command for each intersection
    // each step is initialized as -1 so when we reach -1 we know we are done and the robot stops
    getSteps(numSteps);
    finishedPath = 1;


   // return numSteps;
}

int getPath(int adjM[14][14], int str, int end, int pathShort[25])
{
    if(str==13) // another check to make sure we are not trying to start at B5
    {
        return 0;
    }
    // array which tracks "parent" nodes
    int parent[14]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    // array which tracks which nodes are already in queue
    int inStack[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    // queue for keeping track of reached nodes, initialized at -1, because no node is -1
    int stack[14]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    // initialize starting node as first entry of queue
    stack[0]=str;
    // intialize queue index tracker at 1 after entering first node
    int stackin = 1;
    // initialize past index tracker at 0. The difference will be used to keep track of how many nodes we need to check based on new entries
    int stackinOld = 0;
    // initialize total path stepsat zero
    int numSteps = 0;
    // initialize index difference at zero
    int stackdif =0;
    // initialize checdk array. This will be used to see which nodes need to be checked
    int check[14]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int checkAdj=-1;
    while(1)
    {
        if(stack[stackin]==end) // if statement will be found in nested portions of loop to exit entire loop once destination is found.
        {
            break;
        }
        numSteps++;
        stackdif = stackin-stackinOld;// Calculate the difference in stack indexes between loops.
        for(int i = 0; i<stackdif; i++)
        {
            check[i]=stack[stackinOld+i];// assign un checked nodes in stack into check array
        }

        stackinOld=stackin; // update old index value
        for(int j = 0; j<stackdif; j++)
        {
            if(stack[stackin]==end)
            {
                break;
            }

            // Check adjacency matrix for unchecked values in check array
            checkAdj = check[j];
            for(int k =13; k>=0; k--)
            {
                if(stack[stackin]==end)
                {
                    break;
                }
                // If node in checkAdj is adjacent and not already in the Stack array, add it to stack array and update inStack and parent
                if(adjM[checkAdj][k]==1 && inStack[k]==0)
                {
                    stack[stackin]=k;
                    inStack[k]=1;
                    parent[k]=checkAdj;
                    if(stack[stackin]==end)
                    {
                        break;
                    }
                    stackin++;
                }
            }
        }
    }
    //add end to last entry of shortest path array
    pathShort[numSteps-1]=end;
    //Add the rest of the path steps to pathShort array using parent array
    for(int c = numSteps-2;c>=0;c--)
    {
        pathShort[c]=parent[pathShort[c+1]];
    }
    return numSteps;
    
}
/*
int get2Path(int adjM[14][14], int str, int L1, int L2)
{
    // arrays and variables to store all 4 potential paths   
    int path1[10];
    int path2[10];
    int path3[10];
    int path4[10];
    int pathTemp[20];
    int numStep1;
    int numStep2;
    int numStep3;
    int numStep4;
    int numStepTemp1;
    int numStepTemp2;
    int numstepT;
    int numStepT;
    int flag13 = 0;
    
    // if one destination is B5 we know it must be the second as we cannot legally leave that node
    if(L1==13)
    {
        L1 = L2;
        L2 = 13;
    }
    
    // determine possible options start->1->2 or start->2->1
    // allows for not being concerned which destination was entered in which order
    
    // first determine start -> 1 and 1 -> 2
    path1, numStep1 = getPath(adjM,str,L1,path1);
    path2, numStep2 = getPath(adjM,L1,L2,path2);
    
    numStepTemp1 = numStep1+numStep2;
    for(int i=0;i<numStep1;i++)
    {
    pathTemp[i]=path1[i];
    }
    for(int j=0;j<numStep2;j++)
    {
    pathTemp[j+numStep1]=path2[j];
    }
    if(L2!=13) // check the other variation
    {
        path3, numStep3 = getPath(adjM,str,L2,path3);
        path4, numStep4 = getPath(adjM,L2,L1,path4);
        numStepTemp2 = numStep3+numStep4;
        
        // which option is shorter?
        if(numStepTemp1<numStepTemp2)
        {
            numStepT=numStepTemp1;
            for(int ix = 0; ix<numStepT;ix++)
            {
                path[ix]=pathTemp[ix];
            }
        }
        else
        {
            numStepT=numStepTemp2;
            for(int n = 0; n<numStep3; n++)
            {
                path[n]=path3[n];
            }
            for(int u=0;u<numStep4;u++)
            {
                path[u+numStep3]=path4[u];
            }
        }
    }
    else
    {
        numStepT=numStepTemp1;
        for(int jx = 0;jx<numStepT;jx++)
        {
            path[jx]=pathTemp[jx];
        }
    }

    return numStepT;
}
*/
void getSteps(int numSteps)
{
    int prev=path[0]+1;
    int next=path[0];
    if(next==9)
    {
        steps[0]=1;
    }
    else
    {
        steps[0]=0;
    }
    int cur = next;
    
    for(int m=1;m<numSteps;m++)
    {
        next = path[m];
        if(cur==3&&next-prev==4)
        {
            steps[m]=2;
        }
        else if((cur==12&&next==3)||(cur==5)){steps[m]=5;}
        else if(abs(prev-next)==2 || (abs(prev-next)==4 && cur<5))
        {
            steps[m]=0;
        }
        else if(cur==5||cur==8||cur==9)
        {
            steps[m]=1;
        }
        else if(cur==4)
        {
            steps[m]=3;
        }
        else if(next==prev)
        {
            steps[m]=4;
        }
        else
        {
            steps[m]=1;
        }
        prev = cur;
        cur = next;
    }
}


void operateArm(void *par){
 // Side should be a -1, 1, or 0

 int armPos;
 
 while(1){
  // if(object == 1){ armPos = 1700;} // should be if enemy == 1
  // else { armPos = 0;}
   
   if(enemy){
     activeArm = 1;
   // Write the position to the servo to move the arm
    for(int n = 0; n < 5; n++){
     pause(100);
     servo_angle(armServo,1800);
    }
    
    pause(750);
    
    for(int n = 0; n < 5; n++){
     pause(100);
     servo_angle(armServo,0);
    }
    pause(1000);
    object = 0;  
    activeArm = 0; 
   }
   
   servo_angle(armServo,0);    
   pause(500);    
 }
 
  
}  

void objectWatch(void *par){
 while(1){
    if(object){
     int prevMode = driveMode; 
      
     atObject = 1;
     
     driveMode = -3;

     if(steps[stepIndex-1]!=1){pause(1000);}
     else{pause(500);}
     driveMode = -4;
     
     pause(5000);
     checkSignal =1;
     driveMode = -2;
     
     pause(3000);
     
     int n = 0;
     while((!enemy && !friend) && n < 3000){
      n++; 
     }
     
     if(prevMode != 5){driveMode = prevMode;}
     else{driveMode =0;}
     checkSignal = 0;
     atObject = 0;
     object = 0;
    }
  
   
 }    
  
}  