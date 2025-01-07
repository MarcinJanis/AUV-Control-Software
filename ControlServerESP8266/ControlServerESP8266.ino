#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Artus"; // WiFi name
const char* pass = "ZLKNshiq"; // WiFi password -> change for asking about passwrd and name 

WiFiClient client; //creating object WiFi for establish connection
ESP8266WebServer server(80); // creating Web server object

#define timeDataSend_ms 1000

    String incomingData[8]={"0.0","0.0","0.0","0.0","0.0","0.0","0.0","-1"};
    bool ServerON = false;
    bool chartUpdate = false;

void USART_GetData(){
  int timeStart;
  int waiting_time=0;
  int timeout = 400;
  bool StartOk = false;
  bool EndOk = false;
  if (Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (Serial.available() < 60 || waiting_time < timeout) {
      delay(1); // Wait for full buffer
      waiting_time++;
    }
    //Serial.println("Dane do przeczytania... ");
    String out="";
    while (Serial.available()) {
      char c = Serial.read();
      //Serial.print(c);
      if ( c == '<'){
        out = ""; 
        StartOk = true;
      }
      else if ( c == '>'){
        EndOk = true;
      }
      else if ( c != '>' && EndOk == false){
        out += c;
      }
    }
    
    if (StartOk == true && EndOk == true){
      int searchStart=0;
      int indexOfNext=0;
      for (int i = 0; i<7;i++){
        indexOfNext=out.indexOf(',',searchStart);
        String part = out.substring(searchStart,indexOfNext);
        part.replace("\0", "");
        incomingData[i]=String(part);
        searchStart = indexOfNext+1;
        }
      String part = out.substring(searchStart,searchStart+2);
      incomingData[7]=part;
      }
    else{
      incomingData[7]="CommErr";
      digitalWrite(LED_BUILTIN, LOW);
      }
  }
}


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); // Serial monitor init

  WiFi.begin(ssid, pass); // Connect to WiFi 
  //Connecting to WiFi 
  while(WiFi.status() != WL_CONNECTED) 
  { 
    delay(400);
  }

  // When conected - blink LED
  digitalWrite(LED_BUILTIN, LOW);
  delay(400);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(400);
  digitalWrite(LED_BUILTIN, LOW);
  delay(400);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(400);
  digitalWrite(LED_BUILTIN, LOW);

  // Functions that handle server:
  server.on("/", handleRoot); // Root page
  server.on("/on", handleOn); // Monitoring page
  server.on("/Data", handleData); // sending measurements
  server.begin();
}

void handleRoot()
{  
    String html = "<html><head><style> h1 { font-size: 36px; background-color: darkblue; color: white; text-align: center; margin-top: 20px;}</style></head><body>";
    html += "<h1> AUV control panel </h1>"; 
    html += "<br><br><button onclick=\"location.href='/on'\"> <b>AUV ON</b> </button>";
    html += "<div class=\"footer\">"; 
    html += "<p> Author: Marcin Janis </p>";
    html += "</div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleData() {
  // Sending data from server in json format ( string=" {\"name1\": 123, \"name2\": 456} )
 String jsonResponse = "{";
  jsonResponse += "\"timeSample\":\"" + incomingData[0] + "\",";  // 2 miejsca po przecinku
  jsonResponse += "\"rollSample\":\"" + incomingData[1] + "\",";
  jsonResponse += "\"pitchSample\":\"" + incomingData[2] + "\",";
  jsonResponse += "\"yawSample\":\"" + incomingData[3] + "\",";
  jsonResponse += "\"motorPower\":\"" + incomingData[6] + "\",";
  jsonResponse += "\"servoRightActual\":\"" + incomingData[4] + "\",";
  jsonResponse += "\"servoLeftActual\":\"" + incomingData[5] + "\",";
  jsonResponse += "\"statusMsg\":\"" + incomingData[7] + "\"";
  jsonResponse += "}";
  server.send(200, "application/json", jsonResponse); // Send to panel
}



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
            /* Header 1 - title */
            font-size: 240%; 
            color: white; 
            text-align: center; 
            margin-top: 0%; 
            background-color: darkblue;
            }

          h2 { 
            /* Header 2 */
            font-size: 110%; 
            color: black;  
            margin-top: 3%; 
            }

          h3 {
            /* Header 3 - data box  */
            font-size: 100%;
            color: black; 
	    margin-top: 3%;
            margin-right: 10%; 
            border: 2px solid black; 
            padding: 3%;              
            border-radius: 5px;        
            width: 90%;        
            background-color: #F5F5F5; 
          }

	   h4 {
            /* Header 4 - data box  */
            font-size: 15px;
            color: black; 
	    margin-top: 3%;
            margin-right: 10%; 
            border: 2px solid black; 
            padding: 3%;              
            border-radius: 5px;        
            width: 90%;        
            background-color: #F5F5F5; 
          }

                  
          table {
            width: 60%; /* Width of table */
            border-collapse: collapse;            
	    margin: 5%; /* Space before and after table */
            font-family: Arial, sans-serif; /* Font */
            border-radius: 5px;
            }

            /* Style for table headers */
            th {
              background-color: darkblue; /* Navy background */
              border: 2px solid black; /* Gray borders */
              color: white; /* White font */
              padding: 1%; /* Inner space */
              text-align: center; /* Center text */
	      }

            /* Style for table cells */
            td {
              background-color: lightgrey; /* Light Grey background */
              border: 1px solid black; /* Black borders */
              padding: 1%; /* Inner space */
              text-align: center; /* Center text */
            }

            /* Style for return button */
            .auv-button { 
              margin: 15%;
            }

          /* positioning in two columns */
          .container {
            display: flex; /* flexbox */
            justify-content: space-between;
            margin: 2%; 
          }

          .column {
            width: 42%; 
            padding: 2%; 
            border: 3px solid black; 
            border-radius: 8px; 
          }

          .OffOnButton {
            height: 50%;
            width: 80%;
          }

          .ButtonOn{
            background-color: red; 
            color: white;
          }

          .ButtonOff{
            background-color: green; 
            color: black;
          }

          .button-container {
          justify-content: center;
          display: flex; 
          gap: 3%;     
          }

          </style>
          </head>

          <body>
            <h1> <b> AUV Control Panel </b> </h1>
            <br><br> 
            <div class="button-container">
            	<button onclick="location.href='/'"> <b>AUV OFF</b> </button>
            	<button id="updateTableOnButton"> <b> Log Data </b></button>
            	<button id="motorOnButton"><b> Motor ON/OFF </b></button>
            </div>
            <div class="container">
              <div class="column" > 
                <div class="form-container">
                	<h2> Command Panel: </h2>
                  	<form id="dataForm1">
                    	<h4><label for="selectAction">    Controlled orientation parametr: </label>
                    		<select id="selectAction" name="selectAction">
                      		<option value="R"> Roll </option>
                      		<option value="P"> Pitch </option>
                      		<option value="Y"> Yaw </option>
                      		<option value="I"> Initialization </option>
                    		</select></h4>
                  
                  	<h4><label for="dataInput">Setpoint [deg]: </label>
                  	<input type="text" id="dataInputSetpoint" name="dataInput" placeholder=" ... " required>
                  	<input type="submit" value="Set">
			<br>Values between -180 and 180 are accetable.</h4>
                  	<h4>Permissible error of regulation:<br>
			<label for="dataInput">Setpoint offset [deg]:&emsp;&emsp;&emsp; </label>
                  	<input type="text" id="dataInputSetpointErr1" name="dataInput" placeholder=" ... " required>
                  	<br><label for="dataInput">Ang. Velocity Offset [deg/s]: </label>
                  	<input type="text" id="dataInputSetpointErr2" name="dataInput" placeholder=" ... " required>
                  	<input type="submit" value="Set"></h4>
                </form>
		<hr style="border-width: 2px">
                <form id="dataForm2">

                  <h4><label for="dataInput">Motor Power [%]: </label>
                  <input type="text" id="dataInputMotorPower" name="dataInput" placeholder=" 0 " value="0">
                  <input type="submit" value="Set"></h4>

                  <h4><label for="dataInput">Max Servo Angle [deg] </label>
                  <input type="text" id="dataInputMaxServo" name="dataInput" placeholder=" 30 " value="30">
                  <input type="submit" value="Set"></h4>

                  <h4><label for="dataInput">PID</label><br>
                  <label for="selectReg">Parameters for:&nbsp;</label>
                  <select id="selectReg" name="selectAction"> 
                  <option value="R"> Roll </option>
                  <option value="Y"> Pitch and Yaw </option>
	                </select>&nbsp;regulator<br>
                  P: <input type="text" id="dataInputPID_P" name="dataInput" placeholder="1.37" required> <br>
                  I:&nbsp;  <input type="text" id="dataInputPID_I" name="dataInput" placeholder="0.8" required> <br>
                  D: <input type="text" id="dataInputPID_D" name="dataInput" placeholder="0.45" required> <br>
                  N: <input type="text" id="dataInputPID_N" name="dataInput" placeholder="13.24" required>
                  <br><input type="submit" value="Set"></h4>

                </form>

              </div>
              </div>
              <div class="column" > 
	      <h2> Actual state: </h2>
                <h3>
                  <p id="rollActual"> Roll value [deg]:&nbsp;&nbsp; 0</p>
                  <p id="pitchActual">Pitch value [deg]:  0</p>
                  <p id="yawActual">  Taw value [deg]:&nbsp;&nbsp;  0</p>
		            </h3>
                  <h3><p id="motorActual">Motor Power [%]:  0</p></h3>
                  <h3><p id="stateMsg"> State:  0</p></h3>
              </div>
            </div>
            
            <p id="Table"></p>  
            <table id="MeasurementTable">
                <thead>
                    <tr>
                        <th> Time [s] </th>
                        <th> Roll [deg] </th>
                        <th> Pitch  [deg]  </th>
                        <th> Yaw [deg] </th>
                        <th> Right Servo [deg] </th>
                        <th> Left Servo [deg] </th>
                    </tr>
                </thead>
                <tbody>
                    <!-- Place for rows -->
                </tbody>
            </table>

            <script>
            //  Global Var: 
                var AUV_ON=0;
                var timeSample, rollSample, pitchSample , yawSample , motorPower, statusMsg; // data to send to Panel
                var servoLeftActual, servoRightActual, RollSetPoint, PitchSetPoint, YawSetPoint;
                var rowAmount=0;
                var rowMaxAmount=60;
                var updateTableOn=0;
                var sendDataPrepared =0; // prepared data to send to server

                document.getElementById("motorOnButton").addEventListener("click", function() {
                  AUV_ON = !AUV_ON;

                  if (AUV_ON) {
                    document.getElementById("motorOnButton").innerText = "Stop Motor";
                    document.getElementById("motorOnButton").classList.remove("ButtonOff");
                    document.getElementById("motorOnButton").classList.add("ButtonOn");
                  } else {
                    document.getElementById("motorOnButton").innerText = "Start Motor";
                    document.getElementById("motorOnButton").classList.remove("ButtonOn");
                    document.getElementById("motorOnButton").classList.add("ButtonOff");

                    $.ajax({
                      url: 'http://192.168.100.61/on', 
                      type: "POST",
                      data: { 
                        dataInput: "0"  // Send data
                      },
                      success: function(response) {
                        console.log("Data sent correctly: ", response);
                      },
                      error: function(error) {
                        console.log("Error while sending data: ", error);
                      }
                    });
                  }
                });
              
                document.getElementById("updateTableOnButton").addEventListener("click", function() {
                updateTableOn = !updateTableOn;
                if (updateTableOn) {
                    document.getElementById("updateTableOnButton").innerText = "Data Logger ON";
                    document.getElementById("updateTableOnButton").classList.remove("ButtonOff");
                    document.getElementById("updateTableOnButton").classList.add("ButtonOn");
                  } else {
                    document.getElementById("updateTableOnButton").innerText = "Data Logger OFF";
                    document.getElementById("updateTableOnButton").classList.remove("ButtonOn");
                    document.getElementById("updateTableOnButton").classList.add("ButtonOff");
                  }
                timeSample = 0;
                });
                function updateNumber() {
                    $.ajax({
                        url: '/Data', // Server demand
                        type: 'GET',
                        success: function(data) {
                            console.log("Data received: ", data);
                            //$('#Table').text(data); // Actualisation
                            $('#Table').text(JSON.stringify(data, null, 2));
                            timeSample = data.timeSample; // Set to global var
                            rollSample = data.rollSample;
                            pitchSample = data.pitchSample;
                            yawSample = data.yawSample;
                            motorPower = data.motorPower;
                            servoLeftActual= data.servoLeftActual;
                            servoRightActual= data.servoRightActual;
                            RollSetPoint = data.RollSetPoint;
                            PitchSetPoint = data.PitchSetPoint;
                            YawSetPoint= data.YawSetPoint;
                            statusMsg = data.statusMsg;
                            addRow(); // Add another row
                            displayActual();
                        },
                        error: function(error) {
                          errorCount++;
                          console.log("Error while fetching data:", error);
                           if (errorCount >= 3) {
                           console.log("Stopping retries due to multiple errors.");
                           return;
                        }
                        },
                        complete: function() {
                          setTimeout(updateNumber, 1500);  
                        }
                    });
                }
                updateNumber();
                function addRow() {
                  // Formatting of new row 
                    var col1 = timeSample;
                    var col2 = rollSample;
                    var col3 = pitchSample;
                    var col4 = yawSample;
                    var col5 = servoRightActual;
                    var col6 = servoLeftActual;
                    var newRow = "<tr><td>" + col1 + "</td><td>" + col2 + "</td><td>" + col3 + "</td><td>" + col4 + "</td><td>" + col5 + "</td><td>" + col6 + "</td></tr>";
                  // Adding new row
                  if (updateTableOn == 1){
                    $('#MeasurementTable tbody').append(newRow);
                    rowAmount++;
                    console.log(rowAmount);
                    if (rowAmount>=rowMaxAmount){
                    $('#MeasurementTable tbody tr:first').remove();
                    rowAmount--;
                    }
                    }
                }
                function displayActual(){             
                    document.getElementById("rollActual").innerText = "Roll Value [deg]:  " + rollSample;
                    document.getElementById("pitchActual").innerText = "Pitch Value [deg]: " + pitchSample;
                    document.getElementById("yawActual").innerText = "Yaw Value [deg]:   " + yawSample;
                    document.getElementById("motorActual").innerText = "Motor Power [%]:   " + motorPower;
                    document.getElementById("stateMsg").innerText = "State:  " + statusMsg;
                }
               
                //setInterval(updateNumber, 1000);
                // Command send form 1 - Task managmenet
              $('#dataForm1').on('submit', function(event) {
                event.preventDefault(); // Blocking defoault work 

                var inputCommandSetpoint = $('#dataInputSetpoint').val(); // Get value from input form -> Setpoint
                var selectAction = $('#selectAction').val(); // Pobranie wartości z listy rozwijanej
                var Offset1 = $('#dataInputSetpointErr1').val(); // Pobranie wartości z listy rozwijanej
                var Offset2 = $('#dataInputSetpointErr2').val(); // Pobranie wartości z listy rozwijanej


                // Sending data
                if (AUV_ON == 0){
                  sendDataPrepared = "0";
                }
                else{
                  sendDataPrepared = "O," + selectAction + "," + inputCommandSetpoint+"," + Offset1 + "," + Offset2;
                }
                $.ajax({
                  url: 'http://192.168.100.61/on', 
                  type: "POST",
                  data: { 
                    dataInput: sendDataPrepared  // Send data
                  },
                  success: function(response) {
                    console.log("Data sent correctly: ", response);

                  },
                  error: function(error) {
                    console.log("Error while sending data: ", error);
                  }
                });
              });
              // Command send form 2 - Task managmenet
              $('#dataForm2').on('submit', function(event) {
              event.preventDefault(); // Blocking defoault work 
                var inputCommandMotorPower = $('#dataInputMotorPower').val(); // Get value from input form -> Motor Power
                var inputCommandMaxServo = $('#dataInputMaxServo').val(); // Get value from input form -> Max Servo 
                var inputCommandPID_P = $('#dataInputPID_P').val(); // Get value from input form -> PID_P
                var inputCommandPID_I = $('#dataInputPID_I').val(); // Get value from input form -> PID_I
                var inputCommandPID_D = $('#dataInputPID_D').val(); // Get value from input form -> PID_D
                var inputCommandPID_N = $('#dataInputPID_N').val(); // Get value from input form -> PID_N
                var inputCommandReg = $('#selectReg').val(); 
                // Sending data
                if (AUV_ON == 0){
                  inputCommandMotorPower = 0
                }
                
                sendDataPrepared = "P," + inputCommandReg + "," + inputCommandMotorPower + "," + inputCommandMaxServo + "," + inputCommandPID_P + "," + inputCommandPID_I + "," + inputCommandPID_D + "," + inputCommandPID_N;
                
                $.ajax({
                  url: 'http://192.168.100.61/on', 
                  type: "POST",
                  data: { 
                    dataInput: sendDataPrepared  // Send data
                  },
                  success: function(response) {
                    console.log("Data sent correctly: ", response);

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
      String data_to_send = server.arg("dataInput");
      Serial.print(data_to_send+">");
  }
}




void loop()
{
      //  int timeStart =millis(); // Cyce duration monitoring
        server.handleClient(); // handling HTTP
        USART_GetData(); // If avaliable, Reciving data from STM32
      //  Serial.print("Cycle duration: ");
      //  Serial.println(millis() - timeStart);
}
