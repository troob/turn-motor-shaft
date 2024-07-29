/*
 * Turn Motor Shaft x degrees
 */

//======Advisor======
enum state { moving, resting };

state curState; // track current state

boolean motorStart = false;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoder======
const byte esPin = 3;

int pulsesPerRev = 20;

double minAngularRes,
  pubRotVel = 0.0, // [deg/s], PUBLISHER
  pubRotAccel = 0.0,
  threshRotVel = 0.01, // [deg/s]
  prevRotPos, // [deg]
  setRotVel, // [deg/s]
  setRotAccel;

// values change in callback methods:
volatile long pulseCount,
  prevPulseCount;

volatile double latestRotPos; // [deg]

//======Motor Driver======
const byte mEnablePin = 6,
  mSigPin = 7;

//======Mobile Platform======
double wheelDiam = 6.35; // [cm]

//======Circle======
double piApprox = 3.14159,
  rads = 57.2958; // radians to deg conversion
  
//======Target======
int theta = 180;

//======Controller======
int pubMtrCmd, // PUBLISHER
  outMin,
  outMax,
  pwmPulse;

const int rollingPts = 2;

static unsigned long prevTime,
  prevPidTime;

double kp,
  ki,
  kd,
  errRotVel,
  prevErrRotVel,
  sumErrRotVel,
  threshIntegral,
  errRotAccel,
  prevErrRotAccel,
  sumErrRotAccel;

double prevPubRotVels[rollingPts] = {};
double prevPubRotAccels[rollingPts] = {};

boolean forward = true;

void setup() 
{
  initNode("TurnMotorShaftCW");

  initVars();

  setParams();

  initSubscribers();

  initPublishers();

  /* "spin()" */
  curState = resting;
  
  prevRotPos = latestRotPos;

  prevTime = millis();
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initVars()
{
  setRotVel = 60.0; // [deg/s]
  
  pubMtrCmd = 0;

  pubRotVel = 0.0;

  errRotVel = 0.0;

  prevErrRotVel = 0.0;

  prevRotPos = 0.0;

  latestRotPos = 0.0;

  prevTime = millis();

  prevPulseCount = 0;
}

void setParams()
{
  kp = 1.0;

  ki = 0.0;

  kd = 0.0;
  
  outMin = -255;

  outMax = 255;

  computeMinAngularRes();

  threshRotVel = 0.01;

  threshIntegral = 6.0; // approx from observing that pubMtrCmd b/t ~0-255 below this threshold

  for(int i=0; i < rollingPts; i++)
    prevPubRotVels[i] = 0.0;

  latestRotPos = 0.0; // confusing b/c already set in initVars()

  prevPidTime = millis();

  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();
}

void computeMinAngularRes()
{
  minAngularRes = 360.0 / pulsesPerRev;

  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPin), encoderCallback, CHANGE);
}

void initPublishers()
{
  // motor command
  pinMode(mEnablePin, OUTPUT);
  pinMode(mSigPin, OUTPUT);

  // ADD: publish rotational (and later translational) velocity
}

void stopMoving()
{
  digitalWrite(mEnablePin, LOW);
}

/* "spinOnce()" */
void loop() 
{
  checkUserInput();
  
  // State transitions
  switch(curState)
  {
    case resting: // hold position
      if(motorStart)
      {
        curState = moving;

        prevTime = millis();
      }
      break;

    case moving: // turn CW
      if(!motorStart)
        curState = resting;
      break;
  }

  // State outputs
  switch(curState)
  {
    case resting: // hold position
      holdPosition();
      break;

    case moving: // turn CW
      turnCW(theta);
      break;
  }

  if(Serial.available())
    serialEvent();
}

void checkUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: '");   

    // receive command from user
    if(inputString.substring(0,5) == "start")
    {
      Serial.print("start");
      
      digitalWrite(mSigPin, HIGH); // LOW is CCW looking at motor face
  
      forward = true;
  
      motorStart = true;
    }
    else if(inputString.substring(0,4) == "stop")
    {
      Serial.print("stop");
      
      stopMoving();
  
      motorStart = false;
    }
    else if(inputString.substring(0,10) == "setrotvel ")
      setRotVel = inputString.substring(10, inputString.length()).toFloat(); // get string after 'setrotvel '
    else if(inputString.substring(0,3) == "kp ")
      kp = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kp '
    else if(inputString.substring(0,3) == "ki ")
      ki = inputString.substring(3, inputString.length()).toFloat(); // get string after 'ki '
    else if(inputString.substring(0,3) == "kd ")
      kd = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kd '

    Serial.println("'");
    
    Serial.print("motorStart: ");
    Serial.println(motorStart);

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }
}

void holdPosition()
{
  digitalWrite(mEnablePin, LOW);
}

void turnCW(int angle)
{ 
  if(latestRotPos < angle)
  {
    calcRotVel();

    calcMtrCmd();
  
    analogWrite(mEnablePin, pubMtrCmd); // pubMotor.publish(motor)
  }
  else
    motorStart = false;
}

void calcRotVel()
{
  long dt; // [ms]
  
  double curRotVel;

  dt = millis() - prevTime; // [ms]

  double dtSec = dt / 1000.0;
  
  Serial.print("calcRotVel: dt = ");
  Serial.print(dtSec, 3);
  Serial.print(" s, latestRotPos = ");
  Serial.print(latestRotPos);
  Serial.print(" deg (");
  Serial.print(convertDegToRev(latestRotPos));
  Serial.print(" rev), prevRotPos = ");
  Serial.print(prevRotPos);
  Serial.println(" deg");

  if(dt > 0)
  {
    // we haven't received an updated wheel lately
    if(latestRotPos == prevRotPos)
    {
      curRotVel = 1000.0 * minAngularRes / dt; // if we got a pulse right now, this would be the rot. velocity
  
      // if the velocity is < threshold, consider our velocity 0
      if(abs(curRotVel) < threshRotVel)
      {
        Serial.print("Below threshold: curRotVel=");
        Serial.print(curRotVel);
        Serial.println(" deg/s, pubRotVel = 0.0");
  
        calcRollingVel(0);
      }
      else
      {
        Serial.print("Above threshold: curRotVel = ");
        Serial.print(curRotVel);
        Serial.println(" deg/s");
  
        if(abs(curRotVel) < pubRotVel) // we know we're slower than what we're currently publishing as a velocity
        {
          Serial.println("curRotVel < pubRotVel");
          
          calcRollingVel(curRotVel);
        }
      }
    }
    else // we received a new pulse value
    {
      curRotVel = 1000.0 * ( latestRotPos - prevRotPos ) / dt;

      curRotAccel = ( latestRotVel - prevRotVel ) / dt;
        
      calcRollingVel(curRotVel);

      calcRollingAccel(curRotAccel);
      
      Serial.print("*** Pulse Count Updated: pubRotVel = ");
      Serial.print(pubRotVel);
      Serial.println(" deg/s ***");
    
      prevRotPos = latestRotPos;

      prevRotVel = latestRotVel;
  
      prevTime = millis(); // [ms]
    }
  }
}

double convertDegToRev(double deg)
{
  return deg / 360.0;
}

void calcRollingVel(double val)
{
  for(int i=0; i < rollingPts - 1; i++)
    prevPubRotVels[i] = prevPubRotVels[i + 1];

  prevPubRotVels[rollingPts - 1] = val;

  pubRotVel = getMean(prevPubRotVels);
}

void calcRollingAccel(double val)
{
  for(int i=0; i < rollingPts - 1; i++)
    prevPubRotAccels[i] = prevPubRotAccels[i + 1];

  prevPubRotAccels[rollingPts - 1] = val;

  pubRotAccel = getMean(prevPubRotAccels);
}

double getMean(double vals[])
{
  double sum;
  
  for(int i=0; i < rollingPts; i++)
    sum += vals[i];

  return sum / (double) rollingPts;
}

void calcMtrCmd()
{  
  long pidDt, // [ms]
    dP,
    dI,
    dD;

  double pidDtSec;
  
  pidDt = millis() - prevPidTime;

  prevPidTime = millis();

  pidDtSec = pidDt / 1000.0;
  
  Serial.print("calcMtrCmd: pidDt = ");
  Serial.print(pidDtSec, 3);
  Serial.print(" s, pubRotVel = ");
  Serial.print(pubRotVel);
  Serial.print(" deg/s, setRotVel = ");
  Serial.print(setRotVel);
  Serial.println(" deg/s");
  
  if(pidDt > 0)
  {
    pubMtrCmd = getControlVar(pidDt);
  
    if(pubMtrCmd < 0)
    {
      digitalWrite(mSigPin, LOW);
  
      forward = false;
    }
    else
    {
      digitalWrite(mSigPin, HIGH);
  
      forward = true;
    }

    pubMtrCmd = abs(pubMtrCmd);
  
    Serial.print("pubMtrCmd (0-255, F|B): ");
    Serial.print(pubMtrCmd);
    if(!forward)
      Serial.print(" backward");
    Serial.println("\n");
  }
}

int getControlVar(long dt)
{
  double P, I, D;

  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();

  if(motorStart)
  {
    errRotAccel = setRotAccel - pubRotAccel; // [deg/s^2]
  
    P = kp * errRotAccel;

    if(abs(errRotAccel) < threshIntegral)
      I = ki * sumErrRotAccel;
    else
      I = 0.0;
  
    D = kd * ( errRotAccel - prevErrRotAccel );
  
    pwmPulse += (int) round( P + I + D );
  
    prevErrRotAccel = errRotAccel; // save last (previous) error
  
    sumErrRotAccel += errRotAccel; // sum of error
  }
  else
  {
    errRotAccel = 0.0;

    prevErrRotAccel = 0.0;

    sumErrRotAccel = 0.0;

    pwmPulse = 0;
  }

  // update new velocity
  if(pwmPulse > outMax)
  {
    pwmPulse = outMax;
  }
  else if(pwmPulse < outMin)
  {
    pwmPulse = outMin;
  }

  if(setRotVel == 0.0)
    pubMtrCmd = 0;

  return pwmPulse;
}

//======Interrupt Service Routines======
void encoderCallback()
{
  determinePulseCount(); // increment if forward, decrement if backward // OLD: pulseCount++;
    
  latestRotPos = (double) pulseCount * minAngularRes;
    
  prevPulseCount = pulseCount;
}

void determinePulseCount()
{
  if(forward)
    pulseCount++;
    
  else
    pulseCount--;

  Serial.print("\n\nPulse Count: ");
  Serial.print(pulseCount);
  Serial.println("\n");
}

// desMtrPwrPrcntCallback(?)
void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}
