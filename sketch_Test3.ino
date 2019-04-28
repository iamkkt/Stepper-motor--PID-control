//Mech320-Sensor and Controls Course
//Stepper Motor Control System
const int stepPin = 3;
const int dirPin = 4; 
int customDelay,customDelayMapped; // Defines variables
const int dataIN = 2; //IR sensor INPUT
const int N = 2; // No. of arms/reference points
//PID Paameters
double kp = 2;
double ki = 5;
double kd = 1;
 


unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError,setPoint;
double  output,input,Setpoint;

unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for LCD refresh

int val;
int setrpm;
double rpm; // RPM value

boolean currentstate; // Current state of IR input scan
boolean prevstate; // State of IR sensor in previous scan
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(dataIN,INPUT);     
  prevmillis = 0;
  prevstate = LOW; 
  Serial.begin(115200);
 
  digitalWrite(dirPin,HIGH); //Enables the motor to move in a particular direction
}
void loop() {
  
  customDelayMapped = speedUp(); // Gets custom delay values from the custom speedUp function
  // Makes pules with custom delay, depending on the Potentiometer, from which the speed of the motor depends
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(customDelayMapped);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(customDelayMapped);
  // Measure Actual RPM 
 currentstate = digitalRead(dataIN); // Read IR sensor state
 if( prevstate != currentstate) // If there is change in input
   {
     if( currentstate == HIGH ) // If input changes only from LOW to HIGH
       {
         duration = ( micros() - prevmillis )*N; // Time difference between revolution in microsecond
         prevmillis = micros(); // store time for next revolution calculation
         rpm = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
       }
   }
  prevstate = currentstate; // store this scan (prev scan) data for next scan 
                   
                   //control the motor based on PID value                             
   if( ( millis()-refresh ) >= 100 )
   {     
 Serial.println(rpm); 
  input = analogRead(A0);                //read from rotary encoder connected to A0
        output = computePID(input);
        delay(100);
        analogWrite(3, output);                //control the motor based on PID value
 
   }
}
// Function for reading the Potentiometer
int speedUp() {
  int customDelay = analogRead(A0); // Reads the potentiometer
  int newCustom = map(customDelay, 0, 1023, 300,4000); // Convrests the read values of the potentiometer from 0 to 1023 into desireded delay values (300 to 4000)
  return newCustom;  
  

}
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        static double cumError, rateError;
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
        return out;                                        //have function return the PID output
}
