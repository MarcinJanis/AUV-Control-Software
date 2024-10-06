#include <Wire.h>   // I2C library - not used
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Artus"; // WiFi name
const char* pass = "ZLKNshiq"; // WiFi password -> change for asking about passwrd and name 

WiFiClient client; //creating object WiFi for establish connection
ESP8266WebServer server(80); // creating Web server object

#define PIN_LED 13 // test diode 

 // global variables init
  float RollAngleRegister[] = {0,10,30,-27,15,-48,16,45};
  float TimeBaseRegister[] = {0,1,2,3,4,5,6,7};
  bool chartUpdate = false;


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
  server.on("/", handleRoot); // Root page
  server.on("/on", handleOn); // Monitoring page
  server.on("/Data", handleData); // sending measurements
  server.begin();



  // do usuniecia:
  // Inicjalizacja generatora liczb losowych
  randomSeed(analogRead(0)); // Możesz użyć dowolnego analogowego pinu
}


void handleRoot()
{  
    String html = "<html><head><style> h1 { font-size: 36px; color: blue; text-align: center; margin-top: 20px;}</style></head><body>";
    html += "<h1> AUV control panel </h1>"; 
    html += "<br><br><button onclick=\"location.href='/on'\"> AUV ON </button>";
    html += "<div class=\"footer\">"; // Stopka
    html += "<p> Author: Marcin Janis </p>";
    html += "</div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleData() {
  // Sending data from server in json format ( string=" {\"name1\": 123, \"name2\": 456} )
    String jsonResponse = "{";
    jsonResponse += "\"timeSample\":" + String(random(0, 100)) + ",";
    jsonResponse += "\"rollSample\":" + String(random(100, 200)) + ",";
    jsonResponse += "\"yawSample\":" + String(random(200, 300)) + ",";
    jsonResponse += "\"depthSample\":" + String(random(200, 300)); // Dodany przecinek
    jsonResponse += "}";
    server.send(200, "application/json", jsonResponse);
}

// ...

void handleOn() {
    String html;
    html += R"rawliteral(

        <!DOCTYPE html> 
        <html lang="pl">
        <head>
            <meta charset="UTF-8"> // enable polish letters
            <meta name="viewport" content="width=device-width, initial-scale=1.0">  
            <title>AUV Control Panel</title> 
            <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script> 

            <style>
              h1 { 
              font-size: 36px; 
              color: blue; 
              text-align: center; 
              margin-top: 20px; 
              }
                  
            table {
              width: 50%; /* Width of table */
              border-collapse: collapse; /* Usuwa przerwy między komórkami */
              margin: 20px 0; /* Dodaje odstęp powyżej i poniżej tabeli */
              font-family: Arial, sans-serif; /* Ustala czcionkę */
            }

            /* Styl dla nagłówków tabeli */
            th {
              background-color: #000080; /* Navy background */
              color: white; /* White font */
              padding: 10px; /* Dodaje wewnętrzny odstęp */
              text-align: center; /* Wyrównanie tekstu do lewej */
            }

            /* Styl dla komórek tabeli */
            td {
              border: 1px solid #ddd; /* Szary obramowanie */
              padding: 8px; /* Dodaje wewnętrzny odstęp */
            }
            </style>
        </head>
        <body>
            <h1> AUV Control Panel </h1>
            <br><br><button onclick="location.href='/'">AUV OFF</button> 
            <p id="wynik"></p>  
            <table id="MeasurementTable">
                <thead>
                    <tr>
                        <th> Time [s] </th>
                        <th> Roll [deg] </th>
                        <th> Yaw  [deg]  </th>
                        <th> Depth [m] </th>
                    </tr>
                </thead>
                <tbody>
                    <!-- Place for rows -->
                </tbody>
            </table>

            <script>
            //  Global Var
                var timeSample, rollSample, yawSample, depthSample;
                var rowAmount;
                var rowMaxAmount=10;
                function updateNumber() {
                    $.ajax({
                        url: '/Data', // Server demand
                        type: 'GET',
                        success: function(data) {
                            $('#wynik').text(data); // Aktualizacja wyniku
                            timeSample = data.timeSample; // Przypisanie do globalnych zmiennych
                            rollSample = data.rollSample;
                            yawSample = data.yawSample;
                            depthSample = data.depthSample;
                            addRow(); // Dodanie wiersza po aktualizacji
                        }
                    });
                }
                function addRow() {
                    var col1 = timeSample;
                    var col2 = rollSample;
                    var col3 = yawSample;
                    var col4 = depthSample;
                    // Creating of new row 
                    var newRow = "<tr><td>" + col1 + "</td><td>" + col2 + "</td><td>" + col3 + "</td><td>" + col4 + "</td></tr>";

                    // Dodanie wiersza do tabeli
                    $('#MeasurementTable tbody').append(newRow);
                    rowAmount++;
                    if (rowAmount>=rowMaxAmount){
                    $('#MeasurementTable tbody tr:first').remove();
                    rowAmount--;
                    }
                }

                setInterval(updateNumber, 1000); // Wywoływanie co 1000 ms
            </script>
        </body>
        </html>
    )rawliteral";
    server.send(200, "text/html", html);
}




void loop()
{
  server.handleClient(); // funkcja odpowiadające za obsługę serwera
}
