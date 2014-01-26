#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' the two motors are connected to, In this case, M1 and M2
Adafruit_DCMotor *my_right_Motor = AFMS.getMotor(1);
Adafruit_DCMotor *my_left_Motor = AFMS.getMotor(2);
 
 const int analogrPin = A1; // Analog input pin left
 const int analoglPin = A0; // Analog input pin right
  int sensorValuel = 0; // value left
  int sensorValuer = 0; // value right
  
  int ledl =13; // LED PIN right 13
  int ledr =12; // LED PIN left 12
  
void setup() {
  
  pinMode (ledr, OUTPUT);
  pinMode (ledl, OUTPUT);
    // initialize serial communications at 9600 bps:
  Serial.begin(9600);
   AFMS.begin();  // Init Motorshield
    // initialize Motors
  my_right_Motor->setSpeed(120); //right Motor; 
  my_left_Motor->setSpeed(160); //left Motor different values because Motors are diffents, otherwise same Values as rigth Motor would be correct.
  my_right_Motor->run(FORWARD); 
  my_left_Motor->run(FORWARD);
  // turn off motor
  my_right_Motor->run(RELEASE);
  my_left_Motor->run(RELEASE);
}



void loop() {
  // read analog value of light resistors
  sensorValuer = analogRead(analogrPin);
  sensorValuel = analogRead(analoglPin);
 
 // set LED Pin and motor accoring to the brighter side(LED + Turn towards light)
  if (sensorValuer < sensorValuel) {     // turn left
    digitalWrite(ledr, LOW);
    digitalWrite(ledl,HIGH);    
    my_left_Motor->run(RELEASE);
    my_right_Motor->run(FORWARD); } 
  else {
    // turn left
    digitalWrite(ledr, HIGH);
    digitalWrite(ledl, LOW);    
    my_left_Motor->run(FORWARD);
    my_right_Motor->run(RELEASE); }   

    
 // returns analog values and returns them via serial monitor for debugging
  Serial.print("sensor right = " );   Serial.print(sensorValuer);
  Serial.print("sensor left  = " );   Serial.println(sensorValuel);
 
 // Warte 0,5 Sekunden
  delay(500); }
