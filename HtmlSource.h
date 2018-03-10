/* contain HTML Code of the controller*/



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
