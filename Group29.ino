#include <WiFi.h>
#include <WebServer.h>
#include <Ticker.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// WiFi
const char* ssid = "THE_LIZARD";
const char* password = "12345678";

// Motor pins used
const int LEFT_MOTOR_IN1 = 33;
const int LEFT_MOTOR_IN2 = 32;
const int LEFT_MOTOR_EN = 4;
const int RIGHT_MOTOR_IN3 = 27;
const int RIGHT_MOTOR_IN4 = 26;
const int RIGHT_MOTOR_EN = 2;

// Servo pins
const int UP_SERVO_PIN = 14;
const int DOWN_SERVO_PIN = 12;

// Input pins from Orange Pi P
const int INFRARED_PIN = 5;
const int RADIO_PIN = 18;
const int MAGNETIC_PIN = 19;

// Web server
WebServer server(80);
Ticker controlTicker;

String currentCommand = "stop";

// Variables to store lizard characteristics
String lizardName = "";
String lizardSpecies = "";
int infraredSignal = 0;
int radioSignal = 0;
String magneticSignal = "";

// List to store recorded lizard species
String recordedLizards = "";

// Servo objects
Servo upServo;
Servo downServo;

// Initial positions for servos
int upServoPosition = 90;
int downServoPosition = 90;

// Frequency measurement variables
const int analogPin = 34; // 使用实际的GPIO引脚编号
const int threshold = 300;
unsigned long lastCrossingTime = 0;
unsigned long frequencyStartTime = 0;
int cycleCount = 0;
float frequency = 0.0;
boolean lastState = false;

// Variables for magnetic sensor readings
const int hallSensor1Pin = 35; // 使用实际的GPIO引脚编号
const int hallSensor2Pin = 36; // 使用实际的GPIO引脚编号
const int digitalPin = 4; // 使用实际的GPIO引脚编号
const int numReadings = 10;
int sensorReadings1[numReadings];
int sensorReadings2[numReadings];
int total1 = 0, total2 = 0;
int magneticFieldDifference = 0;
unsigned long lastMagneticReadTime = 0;
const unsigned long magneticReadInterval = 5000; // 5 seconds

// Variables for signal decoding
char name[5];
boolean startReceived = false;
int signalIndex = 0; // 避免与标准库函数冲突
unsigned long lastBitTime = 0;
int bitDuration = 1667; // Microseconds for one bit at 600 bps (1/600 * 1E6)
unsigned long lastSignalCheckTime = 0;
const unsigned long signalCheckInterval = 50; // 50 ms

int magnetCount = 0;
int frequencyCount = 0;
int signalCount = 0;
int pulsecalCount = 0;

// Variables for pulse frequency measurement
const int pulsePin = 2;
const unsigned long measureInterval = 1000;
unsigned long pulseStartTime = 0;
unsigned long pulseCount = 0;
float pulseFrequency = 0;

// Mutex
SemaphoreHandle_t xMutex;

// Function prototypes
void task1(void *pvParameter);
void task2(void *pvParameter);
void measureFrequency();
void analyzeMagneticField();
void measurePulseFrequency();
void decodeSignal();
void initMotorControl();
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void executeCommand();
void readLizardSignals();
void handleRoot();
void handleLizardInfo();
void handleRecordData();
void handleDeleteLastEntry();
void handleUpServoLeft();
void handleUpServoRight();
void handleDownServoLeft();
void handleDownServoRight();
void startDataUpdate();

void setup() {
  Serial.begin(115200);
  initMotorControl();

  upServo.attach(UP_SERVO_PIN);
  downServo.attach(DOWN_SERVO_PIN);

  upServo.write(upServoPosition);
  downServo.write(downServoPosition);

  pinMode(INFRARED_PIN, INPUT);
  pinMode(RADIO_PIN, INPUT);
  pinMode(MAGNETIC_PIN, INPUT);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/lizard_info", handleLizardInfo);
  server.on("/record_data", handleRecordData);
  server.on("/delete_last_entry", handleDeleteLastEntry);
  server.on("/up_servo_left", handleUpServoLeft);
  server.on("/up_servo_right", handleUpServoRight);
  server.on("/down_servo_left", handleDownServoLeft);
  server.on("/down_servo_right", handleDownServoRight);
  server.on("/start_data_update", []() {
    startDataUpdate();
    server.send(204);
  });
  server.on("/forward", []() {
    currentCommand = "forward";
    server.send(204); 
  });
  server.on("/backward", []() {
    currentCommand = "backward";
    server.send(204);  
  });
  server.on("/left", []() {
    currentCommand = "left";
    server.send(204);
  });
  server.on("/right", []() {
    currentCommand = "right";
    server.send(204); 
  });
  server.on("/stop", []() {
    currentCommand = "stop";
    server.send(204);  
  });

  server.begin();
  controlTicker.attach(0.1, executeCommand);

  // Create the mutex
  xMutex = xSemaphoreCreateMutex();

  // Create the tasks
  xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 1, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(task2, "Task2", 4096, NULL, 1, NULL, 0); // Core 0
}

void loop() {
  // Handle the client requests
  server.handleClient();
}

// Task 1 (Core 1): Handle signal processing
void task1(void *pvParameter) {
  for (;;) {
    measureFrequency();
    analyzeMagneticField();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for context switch
  }
}

// Task 2 (Core 0): Handle web server and motor control
void task2(void *pvParameter) {
  for (;;) {
    // Keep server handling in loop
    server.handleClient();
    executeCommand();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for context switch
  }
}
void startDataUpdate() {
  // 定期更新lizard信息
  controlTicker.attach(0.1, readLizardSignals);
}

void measureFrequency() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    int value = analogRead(analogPin);
    boolean currentState = (value > threshold);
    if (currentState && !lastState) {
      unsigned long currentCrossingTime = millis();
      if (cycleCount > 0) {
        unsigned long interval = currentCrossingTime - lastCrossingTime;
        frequency += (1000.0 / interval);
      }
      lastCrossingTime = currentCrossingTime;
      cycleCount++;
    }
    lastState = currentState;
    if (millis() - frequencyStartTime >= 1000) {
      if (cycleCount > 1) {
        frequency /= (cycleCount - 1);
      }
      Serial.print("Frequency: ");
      Serial.print(frequency);
      Serial.println(" Hz");
      cycleCount = 0;
      frequency = 0.0;
      frequencyStartTime = millis();
      frequencyCount++;
    }
    xSemaphoreGive(xMutex);
  }
}

void analyzeMagneticField() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    total1 = 0;
    total2 = 0;
    for (int i = 0; i < numReadings; i++) {
      sensorReadings1[i] = analogRead(hallSensor1Pin);
      sensorReadings2[i] = analogRead(hallSensor2Pin);
      total1 += sensorReadings1[i];
      total2 += sensorReadings2[i];
      delay(10);
    }

    int sensorValue1 = total1 / numReadings;
    int sensorValue2 = total2 / numReadings;
    magneticFieldDifference = sensorValue1 - sensorValue2;

    if (magneticFieldDifference > 10) {
      magneticSignal = "North";
    } else if (magneticFieldDifference < -10) {
      magneticSignal = "South";
    } else {
      magneticSignal = "Neutral";
    }

    Serial.print("Sensor Value 1: ");
    Serial.println(sensorValue1);
    Serial.print("Sensor Value 2: ");
    Serial.println(sensorValue2);
    Serial.print("Magnetic Field Difference: ");
    Serial.println(magneticFieldDifference);
    Serial.print("Magnetic Signal: ");
    Serial.println(magneticSignal);

    lastMagneticReadTime = millis();
    magnetCount++;
    xSemaphoreGive(xMutex);
  }
}

void measurePulseFrequency() {
  unsigned long currentTime = millis();

  if (currentTime - pulseStartTime >= measureInterval) {
    pulseFrequency = (pulseCount / (measureInterval / 1000.0));
    Serial.print("Pulse Frequency: ");
    Serial.print(pulseFrequency);
    Serial.println(" Hz");

    pulseCount = 0;
    pulseStartTime = currentTime;
  }

  if (digitalRead(pulsePin) == HIGH) {
    while (digitalRead(pulsePin) == HIGH);
    pulseCount++;
  }
}

void decodeSignal() {
  Serial.println("Decoding Signal");
  while (signalIndex < 4) { // 4 characters needed
    int incomingByte = 0;
    int incomingBit;

    // Wait for start bit (logic 0)
    if (!startReceived) {
      if (digitalRead(digitalPin) == 0) {
        startReceived = true;
        lastBitTime = micros();
      }
    } else {
      // We are expecting bits now; check timing to read bits
      if (micros() - lastBitTime >= bitDuration) {
        lastBitTime = micros();
        incomingBit = digitalRead(digitalPin);

        // Collect 8 bits for one ASCII character
        for (int i = 0; i < 8; i++) {
          incomingByte |= (incomingBit << i);
          if (i < 7) {
            while (micros() - lastBitTime < bitDuration) {}
            lastBitTime = micros();
            incomingBit = digitalRead(digitalPin);
          }
        }

        // Verify stop bit (logic 1)
        while (micros() - lastBitTime < bitDuration) {}
        lastBitTime = micros();
        if (digitalRead(digitalPin) == 1) {
          if (signalIndex == 0 && incomingByte != 0x23) { // First character must be '#'
            startReceived = false; // Reset for next character
            signalIndex = 0; // Reset index
          } else {
            name[signalIndex++] = incomingByte;
            startReceived = false; // Reset for next character
          }
        } else {
          startReceived = false;
          signalIndex = 0; // Start over if error in transmission
        }
      }
    }
  }

  name[signalIndex] = '\0'; // Null-terminate the string
  Serial.print("Lizard Name: ");
  Serial.println(name);
  signalCount++;
}

void readLizardSignals() {
  // Read analogue signal
  int infraredSignalRaw = analogRead(INFRARED_PIN);
  int radioSignalRaw = analogRead(RADIO_PIN);
  int magneticRawSignal = analogRead(MAGNETIC_PIN);

  // Characteristics Value
  const int infraredAbronia = 571;
  const int radioElgaria = 120;
  const int infraredDixonius = 353;
  const int radioCophotis = 200;

  // Threshold Range to check whether the signal is in the range
  const int threshold = 10;

  // Detect Whether Signal is in the Range
  bool isAbronia = abs(infraredSignalRaw - infraredAbronia) <= threshold && radioSignalRaw < threshold && magneticRawSignal < threshold;
  bool isElgaria = infraredSignalRaw < threshold && abs(radioSignalRaw - radioElgaria) <= threshold && magneticRawSignal < threshold;
  bool isDixonius = abs(infraredSignalRaw - infraredDixonius) <= threshold && radioSignalRaw < threshold && magneticRawSignal > threshold;
  bool isCophotis = infraredSignalRaw < threshold && abs(radioSignalRaw - radioCophotis) <= threshold && magneticRawSignal > threshold;

  // Detect the Species Type
  if (isAbronia) {
    lizardSpecies = "Abronia";
    infraredSignal = infraredAbronia;
    magneticSignal = "N";
  } else if (isElgaria) {
    lizardSpecies = "Elgaria";
    radioSignal = radioElgaria;
    magneticSignal = "N";
  } else if (isDixonius) {
    lizardSpecies = "Dixonius";
    infraredSignal = infraredDixonius;
    magneticSignal = "S";
  } else if (isCophotis) {
    lizardSpecies = "Cophotis";
    radioSignal = radioCophotis;
    magneticSignal = "S";
  } else {
    lizardSpecies = "Unknown";
  }

  lizardName = "#KOA"; 
}

void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Lizard Car Control</title>
      <style>
        body { display: flex; flex-direction: row; justify-content: center; align-items: flex-start; height: 100vh; margin: 0; padding: 20px; box-sizing: border-box; }
        #left-panel, #right-panel { display: flex; flex-direction: column; align-items: center; padding: 20px; }
        #left-panel { flex: 1; }
        #right-panel { flex: 1; }
        #joystick { width: 150px; height: 150px; border: 2px solid black; border-radius: 50%; position: relative; touch-action: none; margin-bottom: 20px; }
        #knob { width: 50px; height: 50px; background: red; border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); }
        .button-box { display: flex; justify-content: center; flex-wrap: wrap; gap: 20px; margin-bottom: 20px; }
        .button-box button { padding: 20px 40px; font-size: 18px; }
        table { border-collapse: collapse; margin-bottom: 20px; }
        table, th, td { border: 1px solid black; padding: 10px; }
        #recorded-list { width: 100%; max-height: 300px; overflow-y: auto; }
        #recorded-list table { width: 100%; }
        #recorded-list th, #recorded-list td { padding: 10px; text-align: left; }
      </style>
    </head>
    <body>
      <div id="left-panel">
        <div id="joystick">
          <div id="knob"></div>
        </div>
        <div class="button-box">
          <button onclick="upServoLeft()">Up Servo Left</button>
          <button onclick="upServoRight()">Up Servo Right</button>
          <button onclick="downServoLeft()">Down Servo Left</button>
          <button onclick="downServoRight()">Down Servo Right</button>
        </div>
      </div>
      <div id="right-panel">
        <table>
          <tr><th>Data Type</th><th>Value</th></tr>
          <tr><td>Infrared Signal</td><td id="infrared">Loading...</td></tr>
          <tr><td>Radio Frequency</td><td id="radio">Loading...</td></tr>
          <tr><td>Magnetic Signal</td><td id="magnetic">Loading...</td></tr>
          <tr><td>Lizard Name</td><td id="name">Loading...</td></tr>
          <tr><td>Lizard Species</td><td id="species">Loading...</td></tr>
        </table>
        <div class="button-box">
          <button onclick="startDataUpdate()">Start Data Update</button>
          <button onclick="recordData()">Record Data</button>
          <button onclick="deleteLastEntry()">Delete Last Entry</button>
        </div>
        <div id="recorded-list">
          <table>
            <thead>
              <tr><th>Recorded Lizards</th></tr>
            </thead>
            <tbody id="recorded-lizards">
              <tr><td>No records yet</td></tr>
            </tbody>
          </table>
        </div>
      </div>
      <script>
        var joystick = document.getElementById('joystick');
        var knob = document.getElementById('knob');
        var isDragging = false;
        var startX, startY, offsetX = 0, offsetY = 0;

        joystick.addEventListener('touchstart', function(e) {
          isDragging = true;
          startX = e.touches[0].clientX - offsetX;
          startY = e.touches[0].clientY - offsetY;
        });

        joystick.addEventListener('touchmove', function(e) {
          if (isDragging) {
            offsetX = e.touches[0].clientX - startX;
            offsetY = e.touches[0].clientY - startY;
            if (offsetX > 50) offsetX = 50;
            if (offsetX < -50) offsetX = -50;
            if (offsetY > 50) offsetY = 50;
            if (offsetY < -50) offsetY = -50;
            knob.style.transform = 'translate(' + offsetX + 'px, ' + offsetY + 'px)';
            sendCommandBasedOnPosition(offsetX, offsetY);
          }
        });

        joystick.addEventListener('touchend', function() {
          isDragging = false;
          offsetX = 0;
          offsetY = 0;
          knob.style.transform = 'translate(-50%, -50%)';
          sendCommand('stop');
        });

        function sendCommandBasedOnPosition(x, y) {
          if (y < -25) sendCommand('forward');
          else if (y > 25) sendCommand('backward');
          else if (x < -25) sendCommand('left');
          else if (x > 25) sendCommand('right');
          else sendCommand('stop');
        }

        function sendCommand(command) {
          fetch('/' + command).catch(function(error) { console.error('Error:', error); });
        }

        function upServoLeft() {
          fetch('/up_servo_left').catch(function(error) { console.error('Error:', error); });
        }

        function upServoRight() {
          fetch('/up_servo_right').catch(function(error) { console.error('Error:', error); });
        }

        function downServoLeft() {
          fetch('/down_servo_left').catch(function(error) { console.error('Error:', error); });
        }

        function downServoRight() {
          fetch('/down_servo_right').catch(function(error) { console.error('Error:', error); });
        }

        function updateLizardInfo() {
          fetch('/lizard_info')
            .then(function(response) { return response.json(); })
            .then(function(data) {
              document.getElementById('infrared').innerText = data.infrared;
              document.getElementById('radio').innerText = data.radio;
              document.getElementById('magnetic').innerText = data.magnetic;
              document.getElementById('species').innerText = data.species;
              document.getElementById('name').innerText = data.name;
            })
            .catch(function(error) { console.error('Error:', error); });
        }

        function recordData() {
          fetch('/record_data').catch(function(error) { console.error('Error:', error); });
        }

        function deleteLastEntry() {
          fetch('/delete_last_entry').catch(function(error) { console.error('Error:', error); });
        }

        function startDataUpdate() {
          setInterval(updateLizardInfo, 5000); // Update every 5 seconds
        }

        updateLizardInfo();
      </script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleLizardInfo() {
  readLizardSignals();

  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    String json = "{\"infrared\": \"" + String(infraredSignal) + " Hz\", \"radio\": \"" + String(radioSignal) + " Hz\", \"magnetic\": \"" + magneticSignal + "\", \"species\": \"" + lizardSpecies + "\", \"name\": \"" + lizardName + "\"}";
    xSemaphoreGive(xMutex);
    server.send(200, "application/json", json);
  }
}

void handleRecordData() {
  recordedLizards += "<tr><td>" + lizardSpecies + "</td></tr>";
  server.send(204);
}

void handleDeleteLastEntry() {
  int lastEntryIndex = recordedLizards.lastIndexOf("<tr>");
  if (lastEntryIndex != -1) {
    recordedLizards = recordedLizards.substring(0, lastEntryIndex);
  }
  server.send(204);
}

void handleUpServoLeft() {
  upServoPosition -= 5;
  if (upServoPosition < 0) upServoPosition = 0;
  upServo.write(upServoPosition);
  server.send(204);
}

void handleUpServoRight() {
  upServoPosition += 5;
  if (upServoPosition > 180) upServoPosition = 180;
  upServo.write(upServoPosition);
  server.send(204);
}

void handleDownServoLeft() {
  downServoPosition -= 5;
  if (downServoPosition < 0) downServoPosition = 0;
  downServo.write(downServoPosition);
  server.send(204);
}

void handleDownServoRight() {
  downServoPosition += 5;
  if (downServoPosition > 180) downServoPosition = 180;
  downServo.write(downServoPosition);
  server.send(204);
}

void initMotorControl() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

void moveForward() {
  analogWrite(LEFT_MOTOR_EN, 255);
  analogWrite(RIGHT_MOTOR_EN, 255);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  Serial.println("F");
}

void moveBackward() {
  analogWrite(LEFT_MOTOR_EN, 255);
  analogWrite(RIGHT_MOTOR_EN, 255);
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  Serial.println("B");
}

void turnLeft() {
  analogWrite(LEFT_MOTOR_EN, 255);
  analogWrite(RIGHT_MOTOR_EN, 255);
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  Serial.println("L");
}

void turnRight() {
  analogWrite(LEFT_MOTOR_EN, 255);
  analogWrite(RIGHT_MOTOR_EN, 255);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  Serial.println("R");
}

void executeCommand() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    if (currentCommand == "forward") {
      moveForward();
    } else if (currentCommand == "backward") {
      moveBackward();
    } else if (currentCommand == "left") {
      turnLeft();
    } else if (currentCommand == "right") {
      turnRight();
    } else {
      stopMotors();
    }
    xSemaphoreGive(xMutex);
  }
}
