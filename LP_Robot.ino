
#define ESP8266WEBSERVER // if not commented out, ESP8266WebServer di ESP8266Webserver.h for NodeMCU permette di pilotare il Robot via Wifi (Veloce)
#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
#define BALANCING  // if not commented out, Balancing is active
#define MANUAL_TUNING 1  // per PID preso dal Pablo
#define CONTROL_PID

// Libraries
#include <Wire.h> // I2C library so we can communicate with the gyro (for the MPU-6050 gyro /accelerometer)
//#include <dummy.h>
//#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library/

// Tabs (header files in sketch directory)
#include "TB6612FNG_DC_Motor_driver.h"
#include "RobotConfig.h"
#include "MPU6050_NODE.h"

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

int Go_mot = 5;       //codificazione della direzione secondo il NumPad
int getstr;
#ifdef CONTROL_PID//************************  PID  *********

#if MANUAL_TUNING
double Kp = 0;                                       //Gain setting for the P-controller
double Ki = 0;                                      //Gain setting for the I-controller 
double Kd = 0;                                       //Gain setting for the D-controller 
#else
double Kp = 12.5;                                       //Gain setting for the P-controller (15)
double Ki = 3.7;                                      //Gain setting for the I-controller (1.5)
double Kd = 0;                                       //Gain setting for the D-controller (30)
double prevKp, prevKi, prevKd;
#endif

//float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150*speedconversionNode;                                //Max target speed (100)
//
//bool isFirstLoop = true;
double originalSetpoint = 0;
double CenterAngleOffset = 4.1;          //angolo con centro di gravita equilibrato
double Setpoint = originalSetpoint-CenterAngleOffset;
//double movingAngleOffset = 0.5;
double Input, Output;//, prevOutput;
double MotorOffset= 1;    // percentuale della velocita per rapportarlo all angolo
//double leftMotorOffset= 1;
//double rightMotorOffset = 1; // to make the robot turn
//int moveState = 0; //0 = balance; 1 = back; 2 = forth
//double roll = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#endif // CONTROL_PID//************************  PID  *********

#ifdef BALANCING //***************************************

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
#endif                      //******************************************

unsigned long loop_timer=0;
double l_timer=1000;
double inizio_loop = 0;
double fine_loop = 0;
double durata_loop = 0;
double inizio_loop_us = 0;
double fine_loop_us = 0;
double durata_loop_us = 0;

const char* ssid = "NoirG";
const char* password = "yakumo99";
 
//int ledPin = 13; // GPIO13

//................WiFi..................

//Parte WebServer piu veloce che quella dell Esempio WifiBlink perche non nel loop
#ifdef ESP8266WEBSERVER
ESP8266WebServer server(80);    //Webserver Object
#endif

//HTML Code.................
const char * htmlMessage = " <!DOCTYPE html> "

"<html>"
  "<body>"

    "<p>This is a HTML only intro page. Please select a button bellow.</p>"
    "<a href=\"/javascript\">Javascript code</a>"
    "</br>"
    "<a href=\"/cssButton\">CSS code</a>"
//parte per pilotare il Robot    
    "<p>Click the buttons to move the Robot:</p>"
    "<button onclick=\"buttonFunction()\">Message</button> "
    "<button onclick=\"buttonFunctionTurnON()\">Turn On</button> "
    "<button onclick=\"buttonFunctionTurnOFF()\">Turn Off</button> "
    "</br></br>"
//------------------------------    Go motor 
    "<a href=\"/Go_LLeft_Forward\"\"><button>Go Left Forward </button></a>"
    "<a href=\"/Go_Forward\"\"><button>Go Straight </button></a>"
    "<a href=\"/Go_RRight_Forward\"\"><button>Go Right Forward </button></a><br />"
    "<a href=\"/Go_Left\"\"><button>Go Left </button></a>"
    "<a href=\"/Stop\"\"><button>Stop Robot </button></a>"
    "<a href=\"/Go_Right\"\"><button>Go Right </button></a><br />"
    "<a href=\"/Go_LLLeft_Backwards\"\"><button>Go Left Backwards </button></a>"
    "<a href=\"/Go_Back\"\"><button>Go Backwards </button></a>"
    "<a href=\"/Go_RRRight_Backwards\"\"><button>Go Right Backwards </button></a><br />"
    "<br><br>"    
    "<a href=\"/KP_UP\"\"><button>KP_UP </button></a>"
    "<a href=\"/KI_UP\"\"><button>KI_UP </button></a>"
    "<a href=\"/KD_UP\"\"><button>KD_UP </button></a>"
    "<a href=\"/CenterAngleOffset_PLUS\"\"><button>CenterAngleOffset_PLUS </button></a><br />"
    "<br><br>"
    "<a href=\"/KP_DOWN\"\"><button>KP_DOWN </button></a>"
    "<a href=\"/KI_DOWN\"\"><button>KI_DOWN </button></a>"
    "<a href=\"/KD_DOWN\"\"><button>KD_DOWN </button></a>"
    "<a href=\"/CenterAngleOffset_MINUS\"\"><button>CenterAngleOffset_MINUS </button></a><br />"
    "<br><br>"
    "<script>"
      "function buttonFunction() { "
        "alert(\"Hello from the ESP8266!\");"
      "} "
      "function buttonFunctionTurnON() { "
        "alert(\"LED ON!\");"
      "} "
      "function buttonFunctionTurnOFF() { "
        "alert(\"LED OFF!\");"
      "} "
      
    "</script>"
    
//end parte per pilotare il Robot   
   
//parte per visualizzare dati
    "<p>Kp:</p> <p id=\"Kp\">""</p>\r\n"   
    "<p>Ki:</p> <p id=\"Ki\">""</p>\r\n"    
    "<p>Kd:</p> <p id=\"Kd\">""</p>\r\n"
    "<p>Setpoint:</p> <p id=\"Setpoint\">""</p>\r\n"   
    "<p>Input:</p> <p id=\"Input\">""</p>\r\n"    
    "<p>Output:</p> <p id=\"Output\">""</p>\r\n"
    "<script>\r\n"
   "var x = setInterval(function() {loadData(\"Kp.txt\",updateKp)}, 1000);\r\n"
   "var x = setInterval(function() {loadData(\"Ki.txt\",updateKi)}, 1000);\r\n"
   "var x = setInterval(function() {loadData(\"Kd.txt\",updateKd)}, 1000);\r\n"
   "var x = setInterval(function() {loadData(\"Setpoint.txt\",updateSetpoint)}, 1000);\r\n"
   "var x = setInterval(function() {loadData(\"Input.txt\",updateInput)}, 1000);\r\n"
   "var x = setInterval(function() {loadData(\"Output.txt\",updateOutput)}, 1000);\r\n"
   "function loadData(url, callback){\r\n"
   "var xhttp = new XMLHttpRequest();\r\n"
   "xhttp.onreadystatechange = function(){\r\n"
   " if(this.readyState == 4 && this.status == 200){\r\n"
   " callback.apply(xhttp);\r\n"
   " }\r\n"
   "};\r\n"
   "xhttp.open(\"GET\", url, true);\r\n"
   "xhttp.send();\r\n"
   "}\r\n"
   "function updateKp(){\r\n"
   " document.getElementById(\"Kp\").innerHTML = this.responseText;\r\n"
   "}\r\n"
      "function updateKi(){\r\n"
   " document.getElementById(\"Ki\").innerHTML = this.responseText;\r\n"
   "}\r\n"
      "function updateKd(){\r\n"
   " document.getElementById(\"Kd\").innerHTML = this.responseText;\r\n"
   "}\r\n"
      "function updateSetpoint(){\r\n"
   " document.getElementById(\"Setpoint\").innerHTML = this.responseText;\r\n"
   "}\r\n"
      "function updateInput(){\r\n"
   " document.getElementById(\"Input\").innerHTML = this.responseText;\r\n"
   "}\r\n"
      "function updateOutput(){\r\n"
   " document.getElementById(\"Output\").innerHTML = this.responseText;\r\n"
   "}\r\n"
   "</script>\r\n"
//fine per visualizzare dati

  "</body> "
"</html> ";

//Javascript Code............ esempio di Java code
const char * javascriptCode = " <!DOCTYPE html> "

"<html>"
  "<body>"
  
    "<p>Click the button to get a message from the ESP8266:</p>"
    "<button onclick=\"buttonFunction()\">Message</button> "
    
    "<script>" //"<REMOVE THIS script>"
    
      "function buttonFunction() { "
        "alert(\"Hello from the ESP8266!\");"
      "} "
      
    "</script>"
  "</body>"
"</html>";

// CSS code .................esempio di CSS code
const char * cssButton ="<!DOCTYPE html>"

"<html>"
    "<head>"
      "<style>"
      
        ".button {"
          "background-color: #990033;"
          "border: none;"
          "color: white;"
          "padding: 7px 15px;"
          "text-align: center;"
          "cursor: pointer;"
        "}"
        
      "</style>"
    "</head>"
    
    "<body>"
      "<input type=\"button\" class=\"button\" value=\"Input Button\">"
     // "<input type=\"button\"class=\"button\"value=\"Input Button\">"
    "</body>"
    
"</html>";

//end Parte WebServer piu veloce che quella dell Esempio WifiBlink
//end ................WiFi..............

//inizio parte per ........Motori.................
//Assegno pin ai motori
MyMotor motor_left(D2, D0, D1, offsetA, D3);   //(pin assegnato a AIN1, AIN2, PWMA, offsetA, STBY)
//MyMotor motor_right(D4, D5, D8, offsetB, D3);   //(pin assegnato a BIN1, BIN2, PWMB, offsetB, STBY)
MyMotor motor_right(D4, D5, D9, offsetB, D3);   //(pin assegnato a BIN1, BIN2, PWMB, offsetB, STBY) Per test interrupt su D8 mpu6050

// Funzioni per muovere il Robot 
void _mForward()
{ 
  motor_right.moveforward(speed);   //valori per la velocita: min 110 meglio di piu fino a max 255
  motor_left.moveforward(speed);
 // Serial.println("go forward!");
}
void _mBack()
{
  motor_right.movebackward(speed);
  motor_left.movebackward(speed);
 // Serial.println("go back!");
}
void _mleft()
{
  motor_right.moveforward(halfspeed);
  motor_left.movebackward(halfspeed);
  Serial.println("go left!");  
}
void _mleftforward()
{
  motor_right.moveforward(speed);
  motor_left.moveforward(halfspeed);
  Serial.println("go fleft!");
}
void _mleftbackward()
{
  motor_right.movebackward(speed);
  motor_left.movebackward(halfspeed);
  Serial.println("go bleft!");
}
void _mright()
{
  motor_right.movebackward(halfspeed);
  motor_left.moveforward(halfspeed);
  Serial.println("go right!");  
}
void _mrightforward()
{
  motor_right.moveforward(halfspeed);
  motor_left.moveforward(speed);
  Serial.println("go fright!");
}
void _mrightbackward()
{
  motor_right.movebackward(halfspeed);
  motor_left.movebackward(speed);
  Serial.println("go bright!");
}
void _mStop()
{
  motor_right.stop();
  motor_left.stop();
 // Serial.println("Stop!");
}
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

  if (Angle_Pitch + CenterAngleOffset > -30.0 && Angle_Pitch + CenterAngleOffset < -2.0) { // Only drive motors, if robot stands upright

     _mBack();
    }
  else if (Angle_Pitch + CenterAngleOffset > 2.0 && Angle_Pitch + CenterAngleOffset < 30) { // Only drive motors, if robot stands upright
    _mForward();
    }
  else {
    _mStop();
    }
}

//
// =======================================================================================================
// BALANCING CALCULATIONS
// =======================================================================================================
//

void balancing() {

  // Read sensor data
  //readMpu6050Data();
  processMpu6050Data();

#ifdef CONTROL_PID//************************  PID  *********
    Setpoint = originalSetpoint-CenterAngleOffset;
        myPID.Compute(); 
   // Input = Angle_Pitch;
    Input = Angle_Pitch+CenterAngleOffset;
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
    
        
#endif // CONTROL_PID//************************  PID  *********

   
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

#ifdef CONTROL_PID//************************  PID  *********    
void setupPid(){
  

 Kp = 30;   //45;                                   
 Ki = 0;                                      
 Kd = 0;      
 
 
/*
  // Speed control loop
  speedPid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  speedPid.SetOutputLimits(-33, 33); // output range from -33 to 33 (same as in balancing() )
  speedPid.SetMode(AUTOMATIC);

  // Angle control loop
  anglePid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  anglePid.SetOutputLimits(-43, 43); // output range from -43 to 43 for motor
  anglePid.SetMode(AUTOMATIC);
*/
    Serial.println("Setup PID");
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(4);             //PID Sample Time 4ms
    myPID.SetOutputLimits(-max_speed, max_speed);
    
    Serial.println("Setup PID done");
}
#endif // CONTROL_PID//************************  PID  *********

#endif      //************************************************

//---------------------------------------------

void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  delay(10);

  //------------------------------  inizio parte per motori
  Serial.println();
  Serial.println("Initializing motors");
  motor_right.initialize();
  motor_left.initialize();
  _mStop();
  Serial.println("Motor Initialized");

   
  //------------------------------  fine parte per motori
  
#ifdef BALANCING    //**********************************************
  if (RobotType == 0) { // Only for self balancing vehicles and cars with MRSC
    // MPU 6050 accelerometer / gyro setup
    //setupMpu6050Register();
    setupMPU6050_NODE_Register();      //MPU6050_NODE file setup MPU6050
    CalibrateMpu6050();
    
#ifdef CONTROL_PID//************************  PID  *********
    // PID controller setup
    setupPid();
#endif // CONTROL_PID//************************  PID  ********* 

                    }
  delay(10);
#endif                          //****************************************************
  
 // pinMode(ledPin, OUTPUT);
 // digitalWrite(ledPin, LOW);
#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER--------- 
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

 //parte wifi comune a ESP8266WiFi.h  e  ESP8266WebServer.h
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {     //Wait for connection
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
#endif                  //--------------------------------------------------------------------ESP8266WEBSERVER---------
  
 //Parte WebServer piu veloce che quella dell Esempio WifiBlink perche non nel loop
#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER---------

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
}
 
void loop() {
  
//double inizio_loop = millis();
 inizio_loop_us = micros();

#ifdef ESP8266WEBSERVER //--------------------------------------------------------------------ESP8266WEBSERVER---------
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
 
