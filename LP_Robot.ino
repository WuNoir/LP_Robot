
// Libraries
#include <Wire.h> // I2C library so we can communicate with the gyro (for the MPU-6050 gyro /accelerometer)
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <Hash.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library/
#include <ArduinoJson.h> 
#include "HtmlSource.h"
// Tabs (header files in sketch directory)
#include "TB6612FNG_DC_Motor_driver.h"
#include "RobotConfig.h"
#include "MPU6050_NODE.h"
#include "FS.h"


//#define ESP8266WEBSERVER // if not commented out, ESP8266WebServer di ESP8266Webserver.h for NodeMCU permette di pilotare il Robot via Wifi (Veloce)
#define USEWIFI 1 // use wifi for robot control
#define DEBUG 0// if not commented out, Serial.print() is active! For debugging only!!
#define BALANCING  1// if not commented out, Balancing is active
#define MANUAL_TUNING 1  // per PID preso dal Pablo
#define CONTROL_PID 1
#define MPU_ADDRESS 0x68
#define JSON_BUFF_DIMENSION 200

#define KPS_FACTOR 

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins

//digital pins Nodemcu corrispondenti ai pin arduin0
///SDA = PIN_WIRE_SDA;
///SCL = PIN_WIRE_SCL;

// LED_BUILTIN = 16;
// BUILTIN_LED = 16;

// D0   = 16; //PWM
// D1   = 5; //PWM
// D2   = 4; //PWM
// D3   = 0; //PWM
// D4   = 2; //PWM
// D5   = 14; //PWM
// D6   = 12; //PWM  SCL
// D7   = 13; //PWM  SDA
// D8   = 15; //PWM
// D9   = 3; //PWM
// D10  = 1; //PWM

//end of digital pins corrispondenti ai pin arduin0


// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1; //non sono ancora usati nel "TB6612FNG_DC_Motor_driver.h"
const int offsetB = 1; //non sono ancora usati nel "TB6612FNG_DC_Motor_driver.h"
//int speedconversionNode=1;     //Da attivare se controllore é l'Arduino
int speedconversionNode=4;     //Da attivare se controllore é il Node 32bit
int speed=100*speedconversionNode;         //valori per la velocita: min 110 meglio di piu fino a max per arduino. 
                                           //Su Node 32bit 4 volte di piu  400 min meglio fino a 1023
                                           //Node a 400 sono circa 10giri/9sec=1.11 giri/sec=399.6°/sec
int halfspeed=speed/2;                    // per girare
int min_speed=40*speedconversionNode;       // 50 su Arduino
int max_speed=255*speedconversionNode;      //255 si Arduino

int max_angle = 20 ;

int Go_mot = 5;       //codificazione della direzione secondo il NumPad
int getstr;


//float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150 * speedconversionNode;                                //Max target speed (100)
//
//bool isFirstLoop = true;

double MotorOffset= 1;    // percentuale della velocita per rapportarlo all angolo
//double leftMotorOffset= 1;
//double rightMotorOffset = 1; // to make the robot turn
//int moveState = 0; //0 = balance; 1 = back; 2 = forth
//double roll = 0;






//int ledPin = 13; // GPIO13

//#####################################################################################
//.................WiFi................................................................
//#####################################################################################

//Parte WebServer piu veloce che quella dell Esempio WifiBlink perche non nel loop

//WiFiManager Wifi;
//#ifdef ESP8266WEBSERVER
#if USEWIFI

String html_home; // the html extracted from flash
StaticJsonBuffer<JSON_BUFF_DIMENSION> jsonBuffer;


WiFiManager wifi;

ESP8266WebServer server(80);    //Webserver Object
WebSocketsServer webSocket(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

            // send message to client
            webSocket.sendTXT(num, "Connected");
        }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
			JsonObject& root = jsonBuffer.parseObject(payload);
			const char* command = root["Command"];
			int Kp = root["KP"];
			int Ki = root["KI"];
			int Kd = root["KD"];
            if(command == "STABILITY") {
                // we get Stability PID data
               UpdateStabilityPID(Kp,Ki,Kd) ;
            }
            else if(command == "THROTTLE") {
                // we get Throttle PID data
               UpdateThrottlePID(Kp,Ki,Kd) ;
            }

            break;
    }

}

void PrepareFile(){
  
  Serial.println("Prepare file system");
  SPIFFS.begin();
  
  File file = SPIFFS.open("/index.html", "r");
  if (!file) {
    Serial.println("file open failed");  
  } else{
    Serial.println("file open success");

    html_home = "";
    while (file.available()) {
      //Serial.write(file.read());
      String line = file.readStringUntil('\n');
      html_home += line + "\n";
    }
    file.close();

    Serial.print(html_home);
  }
}
#endif
//#####################################################################################
//.................PID.................................................................
//#####################################################################################
#if CONTROL_PID

unsigned long loop_timer=0;
double l_timer=1000;
double inizio_loop = 0;
double fine_loop = 0;
double durata_loop = 0;
double inizio_loop_us = 0;
double fine_loop_us = 0;
double durata_loop_us = 0;


// Stability (Mostly PD)..............................................................
double originalSetpointStability = 0;
double CenterAngleOffset = 4.1;          //angolo con centro di gravita equilibrato
double SetpointStability = originalSetpointStability-CenterAngleOffset;
double InputStability, OutputStability;
double KpS = 0;                                       //Gain setting for the P-controller
double KiS = 0;                                      //Gain setting for the I-controller 
double KdS = 0;                                       //Gain setting for the D-controller
double prevKpS, prevKiS, prevKdS;
#if !USEWIFI // balancing for testing
KpS = 30.0;
KiS = 0.0;
KdS = 5.0;
#endif
PID stabilityPID(&InputStability, &OutputStability, &SetpointStability, KpS, KiS, KdS, DIRECT);
void UpdateStabilityPID(int kp, int ki, int kd)
{
  // scale sliders for good value in PID ( experimental )
   
  stabilityPID.SetTunings(KpS, KiS, KdS);
  
  
}


// Throttle (Mostly PI)............................................................
double originalSetpointThrottle = 0;
double SetpointThrottle = originalSetpointThrottle;
double InputThrottle, OutputThrottle;
double KpT = 0;                                       //Gain setting for the P-controller
double KiT = 0;                                      //Gain setting for the I-controller 
double KdT = 0;                                       //Gain setting for the D-controller
double prevKpT, prevKiT, prevKdT;
#if !USEWIFI // balancing for testing
KpT = 30.0;
KiT = 5.0;
KdT = 0.0;
#endif


PID throttlePID(&InputThrottle, &OutputThrottle, &SetpointThrottle, KpT, KiT, KdT, DIRECT);
void UpdateThrottlePID(int kp, int ki, int kd)
{
  // scale sliders for good value in PID ( experimental )
   
  throttlePID.SetTunings(KpT, KiT, KdT);
  
  
}

void setupPids(){

    Serial.println("Setup PID");
    
    stabilityPID.SetMode(AUTOMATIC);
    stabilityPID.SetSampleTime(5);             //PID Sample Time 4ms
    stabilityPID.SetOutputLimits(-max_speed, max_speed);
	throttlePID.SetMode(AUTOMATIC);
    throttlePID.SetSampleTime(5);             //PID Sample Time 4ms
    throttlePID.SetOutputLimits(-max_angle, max_angle);
    
    Serial.println("Setup PID done");
}

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// PI controller implementation (Proportional, integral). DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT * 0.001; // DT is in miliseconds...
  return (output);
}

#endif

//#####################################################################################
//.................Remote Control......................................................
//#####################################################################################


//preso da TheDIYGuy999
// The size of this struct should not exceed 32 bytes
struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Mode1 (toggle speed limitation)
  boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  boolean momentary1 = false; // Momentary push button
  byte pot1; // Potentiometer
};
RcData data;



//#####################################################################################
//.................Balancing...........................................................
//#####################################################################################
long timer_old;
long timer_value;
int debug_counter;
float debugVariable;
float dt;



MPU6050 mpu ( D7, D6 , MPU_ADDRESS );





//#####################################################################################
//.................Motor...............................................................
//#####################################################################################
int16_t motor1;
int16_t motor2;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed



//inizio parte per ........Motori.................
//Assegno pin ai motori
TB6612Motor motor_left(D2, D0, D1, offsetA, D3);   //(pin assegnato a AIN1, AIN2, PWMA, offsetA, STBY)
//MyMotor motor_right(D4, D5, D8, offsetB, D3);   //(pin assegnato a BIN1, BIN2, PWMB, offsetB, STBY)
TB6612Motor motor_right(D4, D5, D8, offsetB, D3);   //(pin assegnato a BIN1, BIN2, PWMB, offsetB, STBY) Per test interrupt su D8 mpu6050

//Funzioni per muovere il Robot 
// void _mForward()
// { 
  // motor_right.Forward(speed);   //valori per la velocita: min 110 meglio di piu fino a max 255
  // motor_left.Forward(speed);
 //Serial.println("go forward!");
// }
// void _mBack()
// {
  // motor_right.Backward(speed);
  // motor_left.Backward(speed);
 //Serial.println("go back!");
// }
// void _mleft()
// {
  // motor_right.Forward(halfspeed);
  // motor_left.Backward(halfspeed);
  // Serial.println("go left!");  
// }
// void _mleftforward()
// {
  // motor_right.Forward(speed);
  // motor_left.Forward(halfspeed);
  // Serial.println("go fleft!");
// }
// void _mleftbackward()
// {
  // motor_right.Backward(speed);
  // motor_left.Backward(halfspeed);
  // Serial.println("go bleft!");
// }
// void _mright()
// {
  // motor_right.Backward(halfspeed);
  // motor_left.Forward(halfspeed);
  // Serial.println("go right!");  
// }
// void _mrightforward()
// {
  // motor_right.Forward(halfspeed);
  // motor_left.Forward(speed);
  // Serial.println("go fright!");
// }
// void _mrightbackward()
// {
  // motor_right.Backward(halfspeed);
  // motor_left.Backward(speed);
  // Serial.println("go bright!");
// }
// void _mStop()
// {
  // motor_right.Stop();
  // motor_left.Stop();
 //Serial.println("Stop!");
// }
  //fine parte per ............Motori............

  
#ifdef BALANCING   //******************************
//
// =======================================================================================================
// "BALANCING" MOTOR DRIVING FUNCTION (for self balancing robot)
// =======================================================================================================
//

void driveMotorsBalancing() {

  // The steering (sterzare/girare) overlay is in degrees per second, controlled by the MPU 6050 yaw rate and yoystick axis 1
//  int steering = ((data.axis1 - 50) / 7) - Yaw_Rate; // -50 to 50 / 8 = 7°/s - yaw_rate
 // steering = constrain(steering, -7, 7); // limita il range di steering between -7 e 7
 // int speed = (angleOutput + 50)*speedconversionNode;

  // Calculate averaged motor power (speed) for speed controller feedback
 // static unsigned long lastSpeed;
 // if (millis() - lastSpeed >= 8) {  // 8ms
 //   speedAveraged = (speedAveraged * 3.0 + angleOutput) / 4.0; // 1:4 (1:8)
 // }

 // speed = constrain(speed, 7*speedconversionNode, 93*speedconversionNode); // same range as in setupPID() + 50 offset from above!

//  if (angleMeasured > -20.0 && angleMeasured < 0.0) { // Only drive motors, if robot stands upright
//    Motor1.drive(speed - steering, minPWM, maxPWMfull, 0, false); // left caterpillar, 0ms ramp! 50 = neutral!
//    Motor2.drive(speed + steering, minPWM, maxPWMfull, 0, false); // right caterpillar
 // }
//  else { // keep motors off
//    Motor1.drive(50, minPWM, maxPWMfull, 0, false); // left caterpillar, 0ms ramp!
//    Motor2.drive(50, minPWM, maxPWMfull, 0, false); // right caterpillar
//  }
/*
  if (mpu.Angle_Pitch + CenterAngleOffset > -30.0 && mpu.Angle_Pitch + CenterAngleOffset < -2.0) { // Only drive motors, if robot stands upright

     _mBack();
    }
  else if (mpu.Angle_Pitch + CenterAngleOffset > 2.0 && mpu.Angle_Pitch + CenterAngleOffset < 30) { // Only drive motors, if robot stands upright
    _mForward();
    }
  else {
    _mStop();
    }
    */
}

//
// =======================================================================================================
// BALANCING CALCULATIONS
// =======================================================================================================
//

void balancing() {
/*
  // Read sensor data
  //mpu.ReadData();
  mpu.ProcessData();

#if CONTROL_PID//************************  PID  *********
    Setpoint = originalSetpoint-CenterAngleOffset;
        myPID.Compute(); 
   // Input = Angle_Pitch;
    Input = mpu.Angle_Pitch+CenterAngleOffset;
    speed = abs(Output*speedconversionNode*MotorOffset);
   // speed = map(speed, 0, 1023, min_speed, max_speed);  //non funzia 
     if (millis() >= l_timer) { 
   Serial.print("Setpoint : "); Serial.print(Setpoint);
   Serial.print("  Input : "); Serial.print(Input); 
   Serial.print("  Output : "); Serial.print(Output); 
   Serial.print("  Kp : "); Serial.print(Kp);
   Serial.print("  Ki : "); Serial.print(Ki);
   Serial.print("  Kd : "); Serial.print(Kd);
   Serial.print("  MotOffset : "); Serial.print(MotorOffset);Serial.print("\n");
     }
     

    if(Serial.available() > 0){
        getstr=Serial.read();
      if(getstr=='q')
      {        Kp +=0.5;      }
      else if(getstr=='a')
      {        Kp -=0.5;      }
      else if(getstr=='w')
      {        Ki +=0.5;      }
      else if(getstr=='s')
      {        Ki -=0.5;      }
      else if(getstr=='e')
      {        Kd +=0.1;      }
      else if(getstr=='d')
      {        Kd -=0.1;      }
      else if(getstr=='r')
      {        MotorOffset +=1;      }
      else if(getstr=='f')
      {        MotorOffset -=1;      }
      else if(getstr=='t')
      {        CenterAngleOffset +=0.1;      }
      else if(getstr=='g')
      {        CenterAngleOffset -=0.1;      }
    }
       myPID.SetTunings(Kp, Ki, Kd);
    
        
*/
   
//  angleMeasured = angle_pitch - tiltCalibration;
/*    inibisco il potentiometer
  // Read speed pot with 0.2s fader
  static unsigned long lastPot;
  if (millis() - lastPot >= 40) { // 40ms
    lastPot = millis();
    speedPot = (speedPot * 4 + data.axis3) / 5; // 1:5
  }
*/
/*
  // PID Parameters (Test)
  double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  //double angleKp = data.pot1 / 8.0, angleKi = 25.0, angleKd = 0.12; // /You need to connect a potentiometer to the transmitter analog input A6
  double angleKp = 200 / 8.0, angleKi = 25.0, angleKd = 0.12; // /You need to connect a potentiometer to the transmitter analog input A6
  // PID Parameters (Working)
  //double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  //double angleKp = data.pot1 / 8.0, angleKi = 25.0, angleKd = 0.12; // You need to connect a potentiometer to the transmitter analog input A6

  // Speed PID controller (important to protect the robot from falling over at full motor rpm!)
  speedTarget = ((float)speedPot - 50.0) / 1.51; // (100 - 50) / 1.51 = Range of about +/- 33 (same as in setupPid() !)
  speedMeasured = speedAveraged * 1.3*speedconversionNode; //angleOutput; // 43 / 33 = 1.3
  speedPid.SetTunings(speedKp, speedKi, speedKd);
  speedPid.Compute();

  // Angle PID controller
  angleTarget = speedOutput / -8.25; // 33.0 (from above) / 8.25 = Range of about +/- 4.0° tilt angle
  //  angleTarget = (speedPot - 50) / -12.5; // 50 / 12.5 = Range of about +/- 4.0° tilt angle
  anglePid.SetTunings(angleKp, angleKi, angleKd);
  anglePid.Compute();

  // Send the calculated values to the motors
  driveMotorsBalancing();

  // Display PID variables on the transmitter OLED (for debugging only, comment out the corresponding variables in checkBattery() in this case)
  //loopDuration(); // compute the loop time
  //payload.vcc = loopTime;
  //payload.vcc = angleMeasured;
  //payload.vcc = speedTarget;
  //payload.batteryVoltage = speedOutput;
  //payload.batteryVoltage = speedMeasured;

  */
}


#endif // CONTROL_PID//************************  PID  *********

//#endif      //************************************************

//---------------------------------------------

void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  delay(10);

  //------------------------------  inizio parte per motori
  Serial.println();
  Serial.println("Initializing motors");
  motor_right.Initialize();
  motor_left.Initialize();
  //_mStop();
  Serial.println("Motor Initialized");

   
  //------------------------------  fine parte per motori
  
#ifdef BALANCING    //**********************************************
  if (RobotType == 0) { // Only for self balancing vehicles and cars with MRSC
    // MPU 6050 accelerometer / gyro setup
    //setupMpu6050Register();
    mpu.Setup_NODE_Register();      //MPU6050_NODE file setup MPU6050
    mpu.Calibrate();
    
#ifdef CONTROL_PID//************************  PID  *********
    // PID controller setup
    setupPids();
#endif // CONTROL_PID//************************  PID  ********* 

                    }
  delay(10);
#endif                          //****************************************************
  
 // pinMode(ledPin, OUTPUT);
 // digitalWrite(ledPin, LOW);
#if USEWIFI
//#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER--------- 
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  
  //read file from spiff
	PrepareFile();
 //parte wifi comune a ESP8266WiFi.h  e  ESP8266WebServer.h
  wifi.autoConnect("AutoConnectAP");
 
  while (WiFi.status() != WL_CONNECTED) {     //Wait for connection
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
     // start webSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    if(MDNS.begin("esp8266")) {
        Serial.println("MDNS responder started");
    }

    // handle index
    server.on("/", []() {
        // send home.html
        server.send(200, "text/html", html_home);
    });
  // Start the server
    server.begin();

    // Add service to MDNS
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
 

  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
#endif                  //--------------------------------------------------------------------ESP8266WEBSERVER---------
//  
// //Parte WebServer piu veloce che quella dell Esempio WifiBlink perche non nel loop
//#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER---------
/*
//parte per visualizzare dati
 server.on("/Kp.txt", [](){
   server.send(200, "text/html", (String)Kp);
 });
 
  server.on("/Ki.txt", [](){
   server.send(200, "text/html", (String)Ki);
 });
 
  server.on("/Kd.txt", [](){
   server.send(200, "text/html", (String)Kd);
 });
 
  server.on("/Setpoint.txt", [](){
   server.send(200, "text/html", (String)Setpoint);
 });
 
  server.on("/Input.txt", [](){
   server.send(200, "text/html", (String)Input);
 });
 
  server.on("/Output.txt", [](){
   server.send(200, "text/html", (String)Output);
 });
 
//end parte per visualizzare dati


server.on("/", []() {                     //Define the handling function for root path (HTML message)
  server.send(200, "text/html", htmlMessage);
});

server.on("/javascript", []() { //Define the handling function for the javascript path
  server.send(200, "text/html", javascriptCode);
});

server.on("/cssButton", []() { //Define the handling function for the CSS path
  server.send(200, "text/html", cssButton);
});
//  Se ricevo il click dei bottoni piloto i motori e ricarico la pagina della radice /
server.on("/Go_LLeft_Forward", []() {
    _mleftforward();
    Serial.print("Left Forward");
    Go_mot = 7;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_Forward", []() {
    _mForward();
    Serial.print("Forward");
    Go_mot = 8;
  server.send(200, "text/html", htmlMessage);
  //server.send(200, "text/plain","<h1>Robot of Node MCU Web Server</h1><h3>Status:</h3> <h4>"+String(Go_mot)+"</h4>");
});

server.on("/Go_RRight_Forward", []() {
    _mrightforward();
    Serial.print("Right Forward");
    Go_mot = 9;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_Left", []() {
    _mleft();
    Serial.print("Left");
    Go_mot = 4;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Stop", []() { //Define the handling function for the Stop path scritta dallo Stop button
  _mStop();
  Serial.print("Stop");
  //Go_mot = 5;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_Right", []() {
    _mright();
    Serial.print("Right");
    Go_mot = 6;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_LLLeft_Backwards", []() {
    _mleftbackward();
    Serial.print("Left Backwards");
    Go_mot = 1;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_Back", []() {
    _mBack();
    Serial.print("Backwards");
    Go_mot = 2;
  server.send(200, "text/html", htmlMessage);
});

server.on("/Go_RRRight_Backwards", []() {
    _mrightbackward();
    Serial.print("Right Backwards");
    Go_mot = 3;
  server.send(200, "text/html", htmlMessage);
});

server.on("/KP_UP", []() {
    Kp +=0.5;
    Serial.println("KP_UP");
  server.send(200, "text/html", htmlMessage);
});

server.on("/KP_DOWN", []() {
    Kp -=0.5;
    Serial.println("KP_DOWN");
  server.send(200, "text/html", htmlMessage);
});

server.on("/KI_UP", []() {
    Ki +=0.5;
    Serial.println("KI_UP");
  server.send(200, "text/html", htmlMessage);
});

server.on("/KI_DOWN", []() {
    Ki -=0.5;
    Serial.println("KI_DOWN");
  server.send(200, "text/html", htmlMessage);
});

server.on("/KD_UP", []() {
    Kd +=0.1;
    Serial.println("KD_UP");
  server.send(200, "text/html", htmlMessage);
});

server.on("/KD_DOWN", []() {
    Kd -=0.1;
    Serial.println("KD_DOWN");
  server.send(200, "text/html", htmlMessage);
});

server.on("/CenterAngleOffset_PLUS", []() {
    CenterAngleOffset +=0.1;
    Serial.println("CenterAngleOffset_PLUS");
  server.send(200, "text/html", htmlMessage);
});

server.on("/CenterAngleOffset_MINUS", []() {
    CenterAngleOffset -=0.1;
    Serial.println("CenterAngleOffset_MINUS");
  server.send(200, "text/html", htmlMessage);
});


server.begin(); //Start the server

Serial.println("Server listening");
#endif                  //--------------------------------------------------------------------ESP8266WEBSERVER---------
 //end Parte WebServer piu veloce che quella dell Esempio WifiBlink perche non nel loop

loop_timer = micros();
*/
}
 
void loop() {
  
  // read information from wifi
  // no need because we are using websockets 
  
  // get timer and dt
  
	timer_value = millis();
    dt = (timer_value - timer_old);
    timer_old = timer_value;
  
  // get robot angle
  
  /*
      angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted = dmpGetPhi();
	*/
  
  // get robot speed 
  
  /*
      actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;  // low pass filter on estimated speed

  */
  
  // get angle from throttle PI 
  /*
      actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;  // low pass filter on estimated speed
*/
  
  // get speed from stability PD
  
  /*
      control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

  */
  
  
//double inizio_loop = millis();
 inizio_loop_us = micros();

#if USEWIFI
//#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER---------
  server.handleClient(); //Handling of incoming requests
#endif                  //--------------------------------------------------------------------ESP8266WEBSERVER---------end


#ifdef BALANCING    //***********************************
// Drive the motors
if (RobotType == 0) {
        balancing(); // Self balancing robot
        driveMotorsBalancing();
       // readMpu6050Data(); 
        }  
#endif                          //***********************************
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop. 
  //Bisogna assicurarsi che il resto del code nel loop non eccede i 4 ms
  
 // if(loop_timer <= micros()) Serial.println("ERROR loop too short !");
  while(loop_timer > micros());
  loop_timer = micros() + LOOP_TIME;
 

 // double fine_loop = millis();
  fine_loop_us = micros();
 // double durata_loop = (fine_loop - inizio_loop);
  durata_loop_us = (fine_loop_us - inizio_loop_us);

   if (l_timer < millis()){
     l_timer =millis()+1000;                                                                       
      }
      
 if (millis() >= l_timer) {                                                                           
     // Serial.print("Loop_mesure: "); Serial.println(durata_loop); 
      Serial.print("Loop_mesure: "); Serial.println(durata_loop_us); 
      }
  

  
 /* //inizio parte per motori
  delay(500);
   _mright();
   Serial.print("destra ");
  delay(1000);
   _mleft();
   Serial.print("sinistra ");
   delay(1000);
   _mStop();
   Serial.print("stop ");
            
  //fine parte per motori
  */
  /*
  loop_mesure = millis() - last_loop_end_time;
  last_loop_end_time = millis();
  Serial.print("Loop time duration");
  Serial.println(last_loop_end_time);
  */

}
 
