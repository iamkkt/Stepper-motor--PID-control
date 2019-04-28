#include <Stepper.h>

#define STEPS 200

Stepper stepper(STEPS, 8, 9, 10, 11);
double kp = 2;
double ki = 5;
double kd = 1;
 
const int dataIN = 2; //IR sensor INPUT
const int N = 2; // No. of arms/reference points

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double  output;

unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
unsigned long refresh; // To store time for LCD refresh

int val;
int setrpm;
double rpm; // RPM value

boolean currentstate; // Current state of IR input scan
boolean prevstate; // State of IR sensor in previous scan

void setup()
{
  pinMode(dataIN,INPUT);     
  prevmillis = 0;
  prevstate = LOW; 
  Serial.begin(9600);
}

void loop()
{
   if (Serial.available()>0)
  {
    
    int sensorReading = analogRead(A0);
    int setrpm = map(sensorReading, 0, 1023, 0, 100);
    setrpm = Serial.parseInt();
    stepper.setSpeed(setrpm);
    
    val = Serial.parseInt();
    stepper.step(val);
   
    
    
  
 
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
                   
        output = computePID(rpm);
        delay(100);
        stepper.setSpeed(output);               //control the motor based on PID value                             
        
 
  
  //  Display
  if( ( millis()-refresh ) >= 100 )
    {
       Serial.println(rpm); 
       Serial.println(output); 
    }

  }
}
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        static double cumError, rateError;
        error = setrpm - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
