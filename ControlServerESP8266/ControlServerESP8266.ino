#include <Wire.h>   // I2C library - not used
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>


const char* ssid = "Artus"; // WiFi name
const char* pass = "ZLKNshiq"; // WiFi password -> change for asking about passwrd and name 

WiFiClient client; //creating object WiFi for establish connection
ESP8266WebServer server(80); // creating Web server object

#define PIN_LED 13 // test diode 

void setup()
{
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200); // Serial monitor init

  WiFi.begin(ssid, pass); // Connect to WiFi 
  Serial.print("Connecting ... "); 

  while(WiFi.status() != WL_CONNECTED) // Connecting animation
  {
    Serial.print("."); 
    delay(400);
    Serial.print("\b "); 
    delay(400);
  }
  
  Serial.println(); // New line
  Serial.print("Connected to: "); // Display connection communicat
  Serial.println( String(ssid) );
  Serial.print("IP: ");
  Serial.println(WiFi.localIP()); // Display IP adress


  // Functions that handle server:
  server.on("/", handleRoot); // main page fcn
  server.on("/on", handleOn); // Vehicle turn on 
  //server.on("/Depth", handleDepth); // Depth control
  //server.on("/DepthSet", handleDepthSet); // Roll control - monitoring
  server.on("/Roll", handleRoll); // Roll control - choosing value
  server.on("/RollSet", handleRollSet); // Roll control - monitoring
  //server.on("/Yaw", handleYaw); // Yaw control
  //server.on("/YawSet", handleYawSet); // Yaw control - monitoring
  server.begin();
}

void handleRoot()
{ 
    digitalWrite(PIN_LED, LOW); // Diod off
    String html = "<html><body>";
    html += "<h1> AUV control panel </h1>";
    html += "<br><br><button onclick=\"location.href='/on'\"> AUV ON </button>";
    html += "<div class=\"footer\">"; // Stopka
    html += "<p> Author: Marcin Janis </p>";
    html += "</div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleOn()
{
  digitalWrite(PIN_LED, HIGH); // Diod on 
  String html = "<html><body>";
  // Title
  html += "<h1> AUV Control Panel </h1>";
  // AUV OFF button (return to Root Screen)
  html += "<br><br><button onclick=\"location.href='/'\"> AUV OFF </button> <br><br>";
  // Choosing parametr, that will be controlled (drop down list) (Depth, Roll, Yaw)
  html += "<label for=\"action\"> Regulated parameter: </label>";
  html += "<select id=\"action\" onchange=\"location = this.value;\">"; // Change on location, choosen from list
  html += "<option value=\"\"> Choose... </option>"; // default Value
  html += "<option value=\"/Depth\"> Depth </option>"; // .../Depth
  html += "<option value=\"/Roll\"> Roll </option>"; // .../Roll
  html += "<option value=\"/Yaw\"> Yaw </option>"; // .../Yaw
  html += "</select>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleRoll()
{
  digitalWrite(PIN_LED, HIGH); // Diod on 
  String html = "<html><body>";
  // Title
  html += "<h1> AUV Control Panel </h1>";
  // Info
  html += "<br><br><br><p> Rotation Control: Roll angle ( X-axis ) </p>";
  // Insert Value
  html += "<form action=\"/RollSet\" method=\"POST\">";
  html += "<input type=\"number\" name=\"RollValue\" step=\"0.01\" placeholder=\"Roll angle [deg]: \" required>";
  html += "<input type=\"submit\" value=\"Send\">";
  html += "</form>";
  html += "<p> Input value range: (-180 , 180 ) [deg] </p> <p> Max two decimal places are accepted </p>";
  // Return button 
  html +="<br><br><br><br>";
  html += "<button onclick=\"location.href='/on'\"> Return </button>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleRollSet() {
  float inputRollValue;

  if (server.hasArg("RollValue")) { // Check if value is recived
    inputRollValue = server.arg("RollValue").toFloat(); // Set var value
    Serial.print("Roll angle request: "); // Serial Monitor display
    Serial.println(inputRollValue); // Serial Monitor display
    String html = "<html><body>";
    // Title
    html += "<h1> AUV Control Panel </h1>";
    html += "<br><br><br>";
    // Return button 
    html += "<p> Roll Value Setpoint: <b>" + String(inputRollValue) + "</b> [deg] </p>"; // Display setpoint value on web 
    html += "<br><br><br>";
    html += "<button onclick=\"location.href='/on'\"> Return </button>"; // Return button
    

  server.send(200, "text/html", html);
  }
  else{ // In case of value not recived
    String html = "<html><body>";
    // Title
    html += "<h1> AUV control panel </h1>";
    html += "<br><br><br>";
    // Return button 
    html += "<p> Error: Roll Value not sent  </p>";
    html += "<br><br><br>";
    html += "<button onclick=\"location.href='/on'\"> Return </button>"; // Return button
  server.send(200, "text/html", html);
  }

}



void loop()
{
  server.handleClient(); // funkcja odpowiadające za obsługę serwera
}