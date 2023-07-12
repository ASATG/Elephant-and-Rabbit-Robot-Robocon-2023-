
//Ardiuno as slave in SPI communication
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

#define motorInterfaceType 1
#define Step_Pin 5
#define Dir_Pin 6  
#define Mic_Step 400

AccelStepper stepper = AccelStepper(motorInterfaceType,Step_Pin,Dir_Pin);


Servo myservo;
int j = 800;
char value;
char prev_value = 'a';
int mode = 0;
int upmode = 0;
int downmode = 0;
int upmode1 = 0;
int downmode1 = 0;
int initialmode = 0;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(4);    // where 4is address in I2c
  Serial.begin(9600);
  myservo.attach(9,800,2200);
  delay(2000);
  myservo.writeMicroseconds(800);
  Wire.onReceive(receiveEvent);
  stepper.setMaxSpeed(3200);
  stepper.setSpeed(3200);
  stepper.setAcceleration(2000);
  //delay(5000);
}

//////With communication
void loop() {
  // put your main code here, to run repeatedly:
  if(j<800)
  {
    j=800;
  }
  if(value != prev_value){
  if(value== '1'){
      upmode = 1;
      // eF(730);
      Serial.println(prev_value); 
      prev_value = '1';
      Serial.println("Go Up");
      Serial.println(value); 
  }
  if(value== '2'){
      downmode = 1;
      Serial.println(prev_value);  
      prev_value = '2';
      Serial.println("Go Down");
      Serial.println(value);
  }
  if(value== '3'){
      mode = 1;
      Serial.println(prev_value); 
      prev_value = '3';
      Serial.println("Down Down");
      Serial.println(value); 
      
  } 
  if(value== '4'){
      mode = 0;
      Serial.println(prev_value); 
      prev_value = '4';
      Serial.println("stop");
      Serial.println(value); 
  } 
  if(value== '5'){
      upmode1 = 1;
      Serial.println(prev_value); 
      prev_value = '5';
      Serial.println("1mmUP");
      Serial.println(value); 
  }
   if(value== '6'){
      downmode1 = 1;
      Serial.println(prev_value); 
      prev_value = '6';
      Serial.println("1mmDOWN");
      Serial.println(value); 
  }  
  // if(value== '7'){
  //     initialmode = 1;
  //     Serial.println(prev_value); 
  //     prev_value = '7';
  //     Serial.println("InitialMode");
  //     Serial.println(value); 
  // }  
  if(value== 'h'){
      j=800;
      prev_value = 'h';
      Serial.println(j);
      Serial.println(value); 
  }
  if(value== 'f'){
    j = 1045;
    prev_value = 'f'; 
    Serial.println(j);
    Serial.println(value); 
  }
  else if(value== 'g'){
    j = 1000;
    prev_value = 'g'; 
    Serial.println(j);
    Serial.println(value);  
  }
  if(value== 'd'){
      j+=2;
      prev_value = 'd';
      Serial.println(j);
      Serial.println(value); 
  }
  else if(value== 'e'){
      j-=2; 
      prev_value = 'e';
      Serial.println(j);
  Serial.println(value);  
  }
  else if(value== 'b')
  {
    j+=5;
    prev_value = 'b'; 
    Serial.println(j);
  Serial.println(value); 
   }
   else if(value== 'c')
  {
    j-=5;
    prev_value = 'c'; 
    Serial.println(j);
    Serial.println(value); 
   }
   else if(value == 'a'){
    prev_value = 'a';
   }

  } 
  myservo.writeMicroseconds(j);
  if(mode == 1){
    stepper.setSpeed(-12800);
    stepper.runSpeed();
  }
  else{
    stepper.setSpeed(0);
    stepper.runSpeed();
  }
  if(upmode == 1){
    eF(698);
    upmode = 0;
  }
  if(downmode == 1){
    eR(-698);
    downmode = 0;
  }
  if(upmode1 == 1){
    eF(183);
    upmode1 = 0;
    upmode = 0;
  }
  if(downmode1 == 1){
    eR(-183);
    downmode1 = 0;
    downmode = 0;
  }
  //  if(initialmode == 1){
  //   eR(8030);
  //   downmode1 = 0;
  //   downmode = 0;
  //   upmode=0;
  //   upmode1=0;
  // }
  
 
}

void receiveEvent(int howMany)
{
  while(Wire.available())
  {
     value = Wire.read();
  }
}


void eF(int a)
{ 
  stepper.setCurrentPosition(0);
  while(stepper.currentPosition() != a)
  {
    stepper.setSpeed(1600);
    stepper.runSpeed();
  }

}
void eR(int a)
{ 
  stepper.setCurrentPosition(0);
  while(stepper.currentPosition() != a)
  {
    stepper.setSpeed(-3200);
    stepper.runSpeed();
  }

}
