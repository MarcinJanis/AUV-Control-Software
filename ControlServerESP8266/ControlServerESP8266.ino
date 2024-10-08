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
    String html = "<html><head><style> h1 { font-size: 36px; background-color: darkblue; color: white; text-align: center; margin-top: 20px;}</style></head><body>";
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
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">  
          <title>AUV Control Panel</title> 
          <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script> 

        <style>
          h1 { 
            font-size: 36px; 
            color: white; 
            text-align: center; 
            margin-top: 20px; 
            background-color: darkblue;
            }

          h2 { 
            font-size: 18px; 
            color: black;  
            margin-top: 10%; 
            }

          h3 {
            font-size: 15px;
            color: black; 
            margin-right: 10%; 
            border: 2px solid black; 
            padding: 3%;              /* Odstęp wewnętrzny */
            border-radius: 5px;        /* Zaokrąglenie rogów */
            width: fit-content;         /* Szerokość dopasowana do zawartości */
            background-color: #F5F5F5; /* Kolor tła */
          }
                  
          table {
            width: 50%; /* Width of table */
            //border-collapse: collapse; /* Delete przerwy (?) */
            margin: 5%; /* Space before and after table */
            font-family: Arial, sans-serif; /* Font */
            border-radius: 5px;
            }

            /* Style for table headers */
            th {
              background-color: darkblue; /* Navy background */
              border: 2px solid black; /* Gray borders */
              color: white; /* White font */
              padding: 10px; /* Inner space */
              text-align: center; /* Center text */
            }

            /* Style for table cells */
            td {
              background-color: lightgrey; /* Light Grey background */
              border: 1px solid black; /* Black borders */
              padding: 8px; /* Inner space */
              text-align: center; /* Center text */
            }

            /* Style for return button */
            .auv-button {
              margin: 15%;
            }

            .container {
            display: flex; /* Używamy flexboxa */
            justify-content: space-between; /* Rozmieszczenie kolumn */
            margin: 20px; /* Odstępy zewnętrzne */
          }

          .column {
            width: 45%; /* Szerokość kolumny */
            padding: 10px; /* Odstęp wewnętrzny */
            border: 1px solid #ccc; /* Ramka kolumny */
            border-radius: 5px; /* Zaokrąglone rogi */
          }

          </style>
          </head>

          <body>
            <h1> <b> AUV Control Panel </b> </h1>
            <br><br> <button onclick="location.href='/'"> AUV OFF </button> <br><br>
            
            <div class="container" >
            <div class="column" > 
            <div class="form-container">
            <h2> Command Panel: </h2>
              <form id="dataForm">
                  
                  <label for="selectAction">    Choose action: </label>
                  <select id="selectAction" name="selectAction">
                  <option value="Roll"> Roll </option>
                  <option value="Pitch"> Pitch </option>
                  <option value="Yaw"> Yaw </option>
                  <option value="Depth"> Depth </option>
                  </select>

                <label for="dataInput"> Inser value: </label>
                <input type="text" id="dataInput" name="dataInput" placeholder=" eg. Roll 30 " required>

                <input type="submit" value="Set">
              </form>

            </div>
            </div>
            <div class="column" > 
            <h3>
              <p id="rollSetpoint">Roll angle [deg]: </p>
              <p id="yawSetpoint"> Yaw angle  [deg]: </p>
              <p id="depthSetpoint">Depth        [m]: </p>
            </h3>
            </div>
            </div>
            <p id="Table"></p>  
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
            //  Global Var: 
                var timeSample, rollSample, yawSample, depthSample; // data to send
                var rowAmount;
                var rowMaxAmount=10;
                function updateNumber() {
                    $.ajax({
                        url: '/Data', // Server demand
                        type: 'GET',
                        success: function(data) {
                            $('#Table').text(data); // Actualisation
                            timeSample = data.timeSample; // Set to global var
                            rollSample = data.rollSample;
                            yawSample = data.yawSample;
                            depthSample = data.depthSample;
                            addRow(); // Add another row
                        }
                    });
                }

                function addRow() {
                  // Formatting of new row 
                    var col1 = timeSample;
                    var col2 = rollSample;
                    var col3 = yawSample;
                    var col4 = depthSample;
                    var newRow = "<tr><td>" + col1 + "</td><td>" + col2 + "</td><td>" + col3 + "</td><td>" + col4 + "</td></tr>";
                  // Adding new row
                    $('#MeasurementTable tbody').append(newRow);
                    rowAmount++;
                    console.log(rowAmount);
                    if (rowAmount>=rowMaxAmount){
                    $('#MeasurementTable tbody tr:first').remove();
                    rowAmount--;
                    }
                }
                setInterval(updateNumber, 1000); // Call function every 1000 ms


                // Command send form 
              $('#dataForm').on('submit', function(event) {
                event.preventDefault(); // Blocking defoault work 

                var inputCommand = $('#dataInput').val(); // Get value from input form
                var selectAction = $('#selectAction').val(); // Pobranie wartości z listy rozwijanej

                // Sending data
                $.ajax({
                  url: 'http://192.168.100.61/on', 
                  type: "POST",
                  data: { 
                    dataInput: selectAction + " " + inputCommand  // Send data
                  },
                  success: function(response) {
                    console.log("Data sent correctly: ", response);
                    /* display setpoints */
                    if (selectAction == "Roll" ){ 
                    document.getElementById("rollSetpoint").innerText = inputCommand;
                    document.getElementById("yawSetpoint").innerText = "0"; 
                    } 
                    if (selectAction == "Yaw" ){ 
                    document.getElementById("rollSetpoint").innerText = "90" ; 
                    document.getElementById("yawSetpoint").innerText = inputCommand; 
                    } 
                    if (selectAction == "Depth" ){ 
                    document.getElementById("depthSetpoint").innerText = inputCommand; 
                    document.getElementById("rollSetpoint").innerText = "0"; 
                    } 

                  },
                  error: function(error) {
                    console.log("Error while sending data: ", error);
                  }
                });
              });


            </script>
        </body>
        </html>
    )rawliteral";

  server.send(200, "text/html", html);

  if (server.hasArg("dataInput")) {
      String receivedData = server.arg("dataInput");
      Serial.println("Otrzymane dane: " + receivedData);
  }
}



void loop()
{
  server.handleClient(); // funkcja odpowiadające za obsługę serwera
}
