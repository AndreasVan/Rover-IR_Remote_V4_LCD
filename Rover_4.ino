
// Control of a Rover5 robot - Last update: AndreasVan 2015-03-13 Version 4.10
// Dagu Rover 5 2WD Tracked Chassis + Explorer Controller Board for Dagu Rover 5 2WD
// Micro controller = Arduino Mega 2560 - 16x2 LCD display Hitachi HD44780
// Detecting obstacles with an SR04 ultrasonic sensor mounted on servo + IR Remote control
// this code is public domain, enjoy!

// Key 1 - turn on robot
// Key 0 - turn off robot
// Key * - Autonomus robot mode
// Key UP - forward
// Key Down - backward
// Key Right - right
// Key Left - left

// LED for direction
// RedLed pin 34 robot drives forward
// GreenLed pin 30 robot drives backward
// YellowLed pin 28 robot turns left
// BlueLed pin 32 robot turns right

// LiquidCrystal
// LCD RS pin to digital pin 50 
// LCD Enable pin to digital pin 52
// LCD D4 pin to digital pin 48
// LCD D5 pin to digital pin 46
// LCD D6 pin to digital pin 44
// LCD D7 pin to digital pin 42
// LCD R/W pin to ground

// SR04 Trigger pin 9  
// SR04 Echo pin 8
// Servo pin 10

// IR Reciver pin 13

#include <IRremote.h>       //library Remote control
#include <Ultrasonic.h>     //library SR04 ultrasonic sensor
#include <Servo.h>          //library Servo (SR04 on servo)
#include <LiquidCrystal.h>  //library LCD Screen

LiquidCrystal lcd(50, 52, 48, 46, 44, 42);
const int PWN1 = 6;       //right PWN
const int DIR1 = 7;       //right DIR
const int PWN2 = 11;      //left PWN
const int DIR2 = 12;      //left DIR
const int TRIGGER=9;      // SR04 sensor 
const int ECHO=8;         // SR04 sensor 
const int redLed = 34;     //robot drives forward
const int greenLed = 30;   //robot drives backward
const int yellowLed = 28;  //robot turns left
const int blueLed = 32;    //robot turns right

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
char choice;
  
//Pin for IR control
int receiver = 38;                 // pin 1 of IR receiver
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results; 
char contcommand;
int modecontrol=0;
int power=0;

const int distancelimit = 30;      //Distance limit for obstacles in front           
const int sidedistancelimit = 15;  //Minimum distance in cm to obstacles at both sides (the robot will allow a shorter distance sideways)

int distance;
int numcycles = 0;
char turndirection; //Gets 'l', 'r' or 'f' depending on which direction is obstacle free
const int turntime = 780; //Time the robot spends turning (miliseconds) // 1200ms c:a 90° - 600ms c:a 45°
int thereis;
Servo head;

void setup(){
  lcd.begin(16, 2);
  Serial.begin (9600);  
  head.attach(10);
  head.write(92);
  irrecv.enableIRIn(); // Start the IR receiver
  pinMode(PWN1, OUTPUT); 
  pinMode(DIR1, OUTPUT); 
  pinMode(PWN2, OUTPUT); 
  pinMode(DIR2, OUTPUT);
  pinMode(TRIGGER,OUTPUT);
  pinMode(ECHO,INPUT);
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  lcd.clear();
  lcd.print("     Rover5");
  lcd.setCursor(0, 1);
  lcd.print("> Version 4.10 < ");
  delay(3000);
  lcd.clear();
  lcd.print("Initiate Rover5");
  lcd.setCursor(0, 1);
  lcd.print("> 1 to start");
  
  //Variable inicialization
  digitalWrite(PWN1,LOW);
  digitalWrite(DIR1,LOW);
  digitalWrite(PWN2,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(TRIGGER,LOW);

}

void go(){ 
   digitalWrite (DIR1, LOW); 
   digitalWrite (DIR2, LOW);
   digitalWrite (PWN1, HIGH);                              
   digitalWrite (PWN2, HIGH); 
   digitalWrite(redLed, HIGH);
   digitalWrite(greenLed, LOW);
   digitalWrite(yellowLed, LOW);
   digitalWrite(blueLed, LOW);
}

void backwards(){
   digitalWrite (DIR1, HIGH); 
   digitalWrite (DIR2, HIGH);
   analogWrite (PWN1 , 90);                              
   analogWrite (PWN2, 90); 
   digitalWrite (redLed, LOW);
   digitalWrite (greenLed, HIGH);
   digitalWrite (yellowLed, LOW);
   digitalWrite (blueLed, LOW);
}

int watch(){
   long interval;
   digitalWrite(TRIGGER,LOW);
   delayMicroseconds(5);                                                                              
   digitalWrite(TRIGGER,HIGH);
   delayMicroseconds(15);
   digitalWrite(TRIGGER,LOW);
   interval=pulseIn(ECHO,HIGH);
   Serial.print(interval);
   interval=interval*0.01657; //how far away is the object in cm  
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Obstacle  cm ");
   lcd.print(centerscanval);
   lcd.setCursor(0, 1);
   lcd.print("> * Remote mode");
   return round(interval);
  

}

void turnleft(int t){
  digitalWrite (DIR1, LOW);    
  digitalWrite (DIR2, HIGH);   
  digitalWrite (PWN1, HIGH);                               
  digitalWrite (PWN2, HIGH);   
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(blueLed, LOW);
  delay(t);
}

void turnright(int t){
  digitalWrite (DIR1, HIGH);    
  digitalWrite (DIR2, LOW);    
  digitalWrite (PWN1, HIGH);                               
  digitalWrite (PWN2, HIGH);   
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, HIGH);
  delay(t);
}  

void stopmove(){
  digitalWrite (DIR1, LOW); 
  digitalWrite (DIR2, LOW);
  digitalWrite (PWN1 ,LOW);                              
  digitalWrite (PWN2, LOW); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}  

void watchsurrounding(){ 
  //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
  //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}
  head.write(120);
  delay(150);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}
  head.write(170); //angle servo (160)
  delay(150); // (300)
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){stopmove();}
  head.write(120);
  delay(150);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}
  head.write(90); // center servo
  delay(150);
  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}
  head.write(40);
  delay(150);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){stopmove();}
  head.write(5);
  delay(150);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){stopmove();}

  head.write(90); //Finish looking around (look forward again)
  delay(150); //(300)
}

char decide(){
  watchsurrounding();
  if (leftscanval>rightscanval && leftscanval>centerscanval){
    choice = 'l';
  }
  else if (rightscanval>leftscanval && rightscanval>centerscanval){
    choice = 'r';
  }
  else{
    choice = 'f';
  }
  return choice;
}

void translateIR() { //Used when robot is switched to operate in remote control mode
    lcd.clear();
    lcd.print("In remote mode !");
    lcd.setCursor(0, 1);
    lcd.print("> * Autonomus");
    
  switch(results.value)
  {
  case 0xFF629D: //Case 'FORWARD'
    go();
    break;
  case 0xFF22DD: //Case 'LEFT'
    turnleft(turntime/2); 
    stopmove();  
    break;
  case 0xFF02FD: //Case 'OK'
    stopmove();   
    break;
  case 0xFFC23D: //Case 'RIGHT'
    turnright(turntime/2);
    stopmove(); 
    break;
  case 0xFFA857: //Case 'REVERSE'
    backwards();
    break;
  case 0xFF42BD:  //Case '*'
    modecontrol=0; stopmove(); // If an '*' is received, switch to automatic robot operating mode
    break;
  default: 
    ;
  }// End Case
  delay(200); // Do not get immediate repeat  
} 

void loop(){
   
  if (irrecv.decode(&results)){ //Check if the remote control is sending a signal
    if(results.value==0xFF6897){ //If an '1' is received, turn on robot
      power=1; }
      lcd.clear();
      lcd.print("booting ...");
      delay(500);
    if(results.value==0xFF4AB5){ //If a '0' is received, turn off robot
      stopmove();
      power=0; }
      lcd.clear();
      lcd.print("Initiate Rover5");
      lcd.setCursor(0, 1);
      lcd.print("Use arrow keys !");
  
      
      
    if(results.value==0xFF42BD){ //If an '*' is received, switch operating mode from automatic robot to remote control (press also "*" to return to automatic robot mode)
      modecontrol=1; //  Activate remote control operating mode
      stopmove(); //The robot stops and starts responding to the user's directions
    }
    irrecv.resume(); // receive the next value
  }
  
  while(modecontrol==1){ //The system gets into this loop during the remote control mode until modecontrol=0 (with '*')
    if (irrecv.decode(&results)){ //If something is being received
      translateIR();//Do something depending on the signal received
      irrecv.resume(); // receive the next value
     }
  }
  if(power==1){
  go();  // if nothing is wrong go forward using go() function above.
  ++numcycles;
  if(numcycles>130){ //Watch if something is around every 130 loops while moving forward 
    watchsurrounding();
    if(leftscanval<sidedistancelimit || ldiagonalscanval<distancelimit){
      turnright(turntime);
    }
    if(rightscanval<sidedistancelimit || rdiagonalscanval<distancelimit){
      turnleft(turntime);
    }
    numcycles=0; //Restart count of cycles
  }
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
    stopmove(); // Since something is ahead, stop moving.
    turndirection = decide(); //Decide which direction to turn.
    switch (turndirection){
      case 'l':
        turnleft(turntime);
        break;
      case 'r':
        turnright(turntime);
        break;
      case 'f':
        ; //Do not turn if there was actually nothing ahead
        break;
    }
    thereis=0;
  }
 }
}
