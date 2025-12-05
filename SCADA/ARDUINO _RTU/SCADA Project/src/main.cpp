/*
 * ESP32 Industrial I/O Module
 * Modbus RTU Slave for PLC Integration
 * Acts like a remote I/O rack
 */
 
//#include <ModbusRTU.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <DHT.h>

// WiFi Configuration
#define WIFI_SSID "GalaxyA12E4D7"         
#define WIFI_PASSWORD "123456789"  

/* n3uron host info
const char* n3uronIP = "192.168.58.253"; // replace with actual n3uron IP
const uint16_t n3uronPort = 502;       // Modbus TCP port n3uron listens on

WiFiClient testClient;
*/

//configure hardware points
#define DHT_PIN 5
#define TRIG_PIN 2
#define ECHO_PIN 4
#define RELAY1_PIN 26
#define RELAY2_PIN 27
#define LED_GREEN 12
#define LED_BLUE 13
#define LED_RED 14
#define ANALOG_IN_1 34  // ADC1_CH6 - Use for voltage/current sensing
#define ANALOG_IN_2 35  // ADC1_CH7 - Use for pressure/flow sensing

//configure modbus - RTU
//#define SLAVE_ID 0
//#define MODBUS_BAUD 9600
//ModbusRTU mb;

//CONFIGURE MODBUS-IP
ModbusIP mb;
/*
IPAddress ESP32_IP(192, 168, 1, 100);      // Set fixed IP for consistency
IPAddress Gateway(192, 168, 1, 1);
IPAddress Subnet(255, 255, 255, 0);
*/

//define DHT sensor
DHT dht(DHT_PIN, DHT11);

//for this module to act as the modbus, registers must be defined
// INPUT REGISTERS (Read-Only - Sensor Data)
//7 input registers are defined to take inputs from sensors
#define IR_TEMP 0           // Temperature x10 (0-600 = 0.0-60.0°C)
#define IR_HUMIDITY 1       // Humidity x10 (0-1000 = 0.0-100.0%)
#define IR_OIL_LEVEL 2      // Oil level in cm (0-500)
#define IR_DI_STATUS 3      // Digital inputs status (bits)
#define IR_AI_RAW_1 4       // Raw analog input 1
#define IR_AI_RAW_2 5       // Raw analog input 2
#define IR_MODULE_STATUS 6  // Module health status

// HOLDING REGISTERS (Read/Write - Control & Configuration)
//these store and pass data to display
#define HR_DO_CMD 0         // Digital outputs command (bit 0=Relay1, bit 1=Relay2)
#define HR_LED_CMD 1        // LED control (bits 0=Green, 1=Yellow, 2=Red)
#define HR_ALARM_ACK 2      // Alarm acknowledge
#define HR_MODULE_MODE 3    // Operating mode (0=Auto, 1=Manual)
#define HR_TEMP_SETPOINT 4  // Temperature setpoint x10
#define HR_HYS_SETPOINT 5   // Hysteresis setpoint x10

// Module status bits
//these registers allocated for monitoring overall system status
#define STATUS_OK 0x0001
#define STATUS_SENSOR_FAULT 0x0002
#define STATUS_COMM_ERROR 0x0003
#define STATUS_ALARM_ACTIVE 0x0004
#define STATUS_MANUAL_MODE 0x0010
#define STATUS_TEST_MODE 0x0020

// Operating modes
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_TEST 2

// Timing
//timing variables required for processing
unsigned long lastSensorRead = 0;
unsigned long lastCommCheck = 0;
const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long COMM_TIMEOUT = 5000;
const unsigned long LED_UPDATE_INTERVAL = 100;
// Variables for detection
unsigned long lastTriggerTime = 0;
int triggerCount = 0;

// Data storage
uint16_t inputRegisters[20] = {0};
uint16_t holdingRegisters[20] = {0};
uint16_t moduleStatus = STATUS_OK;


// NEW: Temperature control variables
bool heatingActive = false;
bool coolingActive = false;
unsigned long lastTempControl = 0;
const unsigned long TEMP_CONTROL_INTERVAL = 2000;

// Function Prototypes
//CONNECTIVITY FUNCTIONS
void connectWiFi();
//void testN3uronConnection();
void setupModbusTCP();

//CODE FUNCTIONS
void readSensors();
void readAnalogInputs();
void updateModbusInputs();
void processOutputCommands();
void printRawRegisters();
//new
void processTemperatureControl();
void processLEDControl();
void processTestMode();
//
void checkCommunication();
void updateModuleStatus();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.println("ESP32 Industrial I/O Module Starting...");
  Serial.println("\n\nESP32 Industrial I/O Module - Modbus TCP (WiFi) Starting...");
  

  
  // Initialize Modbus RTU Slave - 0 (point of measurement in the overall supervisory system)
  //Serial2.begin(MODBUS_BAUD, SERIAL_8N1, 16, 17); // RX=16, TX=17
  //mb.begin(&Serial2);
  //mb.slave(SLAVE_ID);
  
  // Register Modbus areas
  //input and holding registers are setup
 // mb.addIreg(0, 0, 20);  // Input Registers (sensors)
 // mb.addHreg(0, 0, 20);  // Holding Registers (control)
  
  // Initialize sensors - dht sensor
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // NEW: Initialize analog inputs
  pinMode(ANALOG_IN_1, INPUT);
  pinMode(ANALOG_IN_2, INPUT);
  
  // Initialize outputs
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Default state
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(LED_GREEN, HIGH); // System ready
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);

  // Connect to WiFi
  connectWiFi();

  //testN3uronConnection();
  
  // Setup Modbus TCP
  setupModbusTCP();
  
  Serial.println("✓ I/O Module Ready");
  //Serial.printf("  Modbus Slave ID: %d\n", SLAVE_ID);
//  Serial.printf("  Baud Rate: %d\n", MODBUS_BAUD);
  Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
  Serial.printf("  Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.printf("  MAC Address: %s\n", WiFi.macAddress().c_str());
  //Serial.printf("  Modbus TCP: Listening on port %u\n", n3uronPort);
  Serial.println("  Waiting for N3URON connection...\n");
}


void loop() {
  // put your main code here, to run repeatedly:
  // Handle Modbus communication - started continuous communication link
  mb.task();
  
  // Check communication timeout
  checkCommunication();
  

  
  // Read sensors periodically
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    readSensors();
    readAnalogInputs();
    updateModbusInputs();
    printRawRegisters();
  }
  
  // Process scada commands
  processOutputCommands();
  processLEDControl();

  if(millis()- lastTempControl >= TEMP_CONTROL_INTERVAL) {
    lastTempControl = millis();
    processTemperatureControl();
  }



  uint16_t mode = mb.Hreg(HR_MODULE_MODE);
  if (mode == MODE_TEST) {
    processTestMode();
  }

  // Update module status
  updateModuleStatus();
}

/*void testN3uronConnection() {
if (testClient.connect(n3uronIP, n3uronPort)) {
  Serial.println("✓ N3URON reachable!");
  testClient.stop(); // Close connection after test
} else {
  Serial.println("✗ N3URON NOT reachable!");
}
//delay(5000); // check every 5 seconds
}
*/

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
//  WiFi.config(ESP32_IP, Gateway, Subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n WiFi Connected!");
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_GREEN, HIGH);  // WiFi connected
    digitalWrite(LED_BLUE, LOW);
  } else {
    Serial.println("\n WiFi Connection Failed!");
    digitalWrite(LED_RED, HIGH);
    moduleStatus |= STATUS_COMM_ERROR;
  }
}

void setupModbusTCP() {
  mb.server(1);  // IMPORTANT: Set Unit ID to 1 to match N3uron config
  mb.begin();
  
  // Register Modbus areas
  mb.addIreg(0, 0, 20);   // Input Registers (sensors)
  mb.addHreg(0, 0, 20);   // Holding Registers (control)

  // Initialize all registers to zero
  for (int i = 0; i < 20; i++) {
    mb.Ireg(i, 0);
    mb.Hreg(i, 0);
  }
  
  Serial.println(" Modbus TCP Server Started");
  Serial.println(" Unit ID: 1");
  Serial.println(" Input Registers: 0-19");
  Serial.println(" Holding Registers: 0-19");

}




void readSensors() {
  // Read DHT22
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();


  
  //dht sensor error handling
  if (isnan(temp) || isnan(humidity)) {
    moduleStatus |= STATUS_SENSOR_FAULT;
    inputRegisters[IR_TEMP] = 0;
    inputRegisters[IR_HUMIDITY] = 0;
    Serial.println(" DHT11 Read Error");
  } else {
    //dht sensor pass output
    moduleStatus &= ~STATUS_SENSOR_FAULT;
    inputRegisters[IR_TEMP] = (uint16_t)(temp * 10);
    inputRegisters[IR_HUMIDITY] = (uint16_t)(humidity * 10);

    Serial.printf("Stored: IR_TEMP=%d, IR_HUMIDITY=%d\n", 
                  inputRegisters[IR_TEMP], inputRegisters[IR_HUMIDITY]);
  }

  // Read Ultrasonic
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //disntance calc
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = duration * 0.034 / 2;


  //define readable distance range
  if (distance > 0 && distance < 400) {
    inputRegisters[IR_OIL_LEVEL] = (uint16_t)distance;
  }
  
  Serial.printf("Stored: IR_OIL_LEVEL=%d,\n",
                  inputRegisters[IR_OIL_LEVEL]);
  // Digital inputs status (simulated - could read from GPIO)
  inputRegisters[IR_DI_STATUS] = 0x00; // All DI off for now

  // Update module status
  inputRegisters[IR_MODULE_STATUS] = moduleStatus;
}

void readAnalogInputs() {
  // Read Analog Input 1
  int analogValue1 = analogRead(ANALOG_IN_1);
  inputRegisters[IR_AI_RAW_1] = (uint16_t)analogValue1;
  
  // Read Analog Input 2
  int analogValue2 = analogRead(ANALOG_IN_2);
  inputRegisters[IR_AI_RAW_2] = (uint16_t)analogValue2;
  
  Serial.printf("Stored: IR_AI_RAW_1=%d, IR_AI_RAW_2=%d\n",
                inputRegisters[IR_AI_RAW_1],
                inputRegisters[IR_AI_RAW_2]);
}


void updateModbusInputs() {
  // Update Input Registers from local storage
  for (int i = 0; i < 20; i++) {
    mb.Ireg(i, inputRegisters[i]);
  }

/*
  // Log sensor values occasionally for debugging
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 10000) {  // Every 10 seconds
    lastLog = millis();
    Serial.printf("📊 Sensors: Temp=%.1f°C, Humidity=%.1f%%, Distance=%dcm\n",
                  mb.Ireg(0) / 10.0,
                  mb.Ireg(1) / 10.0,
                  mb.Ireg(2));
  }
  */

  // Update module status register
  inputRegisters[IR_MODULE_STATUS] = moduleStatus;
  mb.Ireg(IR_MODULE_STATUS, moduleStatus);

  //DEBUG CODE:
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 2000) {  // Every 10 seconds
    lastLog = millis();
      Serial.println("\n=== MODBUS REGISTERS ===");
      Serial.println("\n=== INPUT REGISTERS ===");
      Serial.printf("Register 0 (Temp):     %d (%.1f°C)\n", mb.Ireg(0), mb.Ireg(0)/10.0);
      Serial.printf("Register 1 (Humidity): %d (%.1f%%)\n", mb.Ireg(1), mb.Ireg(1)/10.0);
      Serial.printf("Register 2 (Distance): %d cm\n", mb.Ireg(2));
      Serial.printf("Register 6 (Status):   0x%04X\n", mb.Ireg(6));
  Serial.println("========================\n");
  }
}

void processOutputCommands() {
  // Read commands from PLC (Holding Registers) - LEDs and Relays are read/write
  uint16_t doCmd = mb.Hreg(HR_DO_CMD);
  uint16_t ledCmd = mb.Hreg(HR_LED_CMD);
  
  // Control Relays (Digital Outputs)
  digitalWrite(RELAY1_PIN, (doCmd & 0x01) ? HIGH : LOW);
  digitalWrite(RELAY2_PIN, (doCmd & 0x02) ? HIGH : LOW);
  
  // Control LEDs
  digitalWrite(LED_GREEN, (ledCmd & 0x01) ? HIGH : LOW);
  digitalWrite(LED_BLUE, (ledCmd & 0x02) ? HIGH : LOW);
  digitalWrite(LED_RED, (ledCmd & 0x04) ? HIGH : LOW);
  
  // Log changes
  static uint16_t lastDoCmd = 0;
  if (doCmd != lastDoCmd) {
    Serial.printf(" Manual_Output: DO=0x%04X (Relay1=%d, Relay2=%d)\n", 
                  doCmd, (doCmd & 0x01), (doCmd & 0x02) >> 1);
    lastDoCmd = doCmd;
  }
}

void printRawRegisters() {
    Serial.print("IREG ");
    for (int i = 0; i < 7; i++) {
        Serial.print(mb.Ireg(i));
        Serial.print(" ");
    }
    Serial.print(" | HREG ");
    for (int i = 0; i < 7; i++) {
        Serial.print(mb.Hreg(i));
        Serial.print(" ");
    }
    Serial.println();
}


void processLEDControl() {

  uint16_t ledcmd = mb.Hreg(HR_LED_CMD);
  uint16_t mode = mb.Hreg(HR_MODULE_MODE);

  if (mode == MODE_MANUAL || mode == MODE_TEST) {

    //manually control leds
    digitalWrite(LED_GREEN, (ledcmd & 0x01) ? HIGH : LOW);
    digitalWrite(LED_BLUE, (ledcmd & 0x02) ? HIGH : LOW);
    digitalWrite(LED_RED, (ledcmd & 0x04) ? HIGH : LOW);
    moduleStatus |= STATUS_MANUAL_MODE;
  } else {
    //auto led control
    if (moduleStatus & STATUS_COMM_ERROR) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
    } else if (moduleStatus & STATUS_ALARM_ACTIVE) {
      //ALARM INDICATION
      digitalWrite(LED_RED, (millis()/500) % 2); // Blink Red LED
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
    } else if (moduleStatus & STATUS_SENSOR_FAULT) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, HIGH);
    } else {
      //ALL GOOD
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
    }
  }
}

  //SETPOINT AND HYSTERISIS FOR TEMP CONRTROL

  void processTemperatureControl() {

    uint16_t mode = mb.Hreg(HR_MODULE_MODE);
    if (mode != MODE_AUTO) {
      heatingActive = false;
      coolingActive = false;
      return;
    }

    float currentTemp = inputRegisters[IR_TEMP] / 10.0;
    float setpoint = mb.Hreg(HR_TEMP_SETPOINT) / 10.0;
    float hysteresis = mb.Hreg(HR_HYS_SETPOINT) / 10.0;

    // Simple on/off temperature control with hysteresis
  // Relay 1 = Heating (turns on when temp < setpoint - hysteresis)
  // Relay 2 = Cooling (turns on when temp > setpoint + hysteresis)
  
  if (currentTemp < (setpoint - hysteresis)) {
    heatingActive = true;
    coolingActive = false;
    digitalWrite(RELAY1_PIN, HIGH); // Turn on heating
    digitalWrite(RELAY2_PIN, LOW);  // Turn off cooling
  } 
  else if(currentTemp > (setpoint + hysteresis)) {
    heatingActive = false;
    coolingActive = true;
    digitalWrite(RELAY1_PIN, LOW);  // Turn off heating
    digitalWrite(RELAY2_PIN, HIGH); // Turn on cooling
  } else if(currentTemp >= setpoint - hysteresis/2 && currentTemp <= setpoint + hysteresis/2){
    heatingActive = false;
    coolingActive = false;
    digitalWrite(RELAY1_PIN, LOW);  // Turn off heating
    digitalWrite(RELAY2_PIN, LOW);  // Turn off cooling
  }

    static bool lastHeatingState = false;
    static bool lastCoolingState = false;
    
    if (heatingActive != lastHeatingState|| coolingActive != lastCoolingState) {
      Serial.printf("🌡️  AUTO Temp Control: Temp=%.1f°C, SP=%.1f°C, Heat=%s, Cool=%s\n",
                  currentTemp, setpoint, 
                  heatingActive ? "ON" : "OFF",
                  coolingActive ? "ON" : "OFF");
    lastHeatingState = heatingActive;
    lastCoolingState = coolingActive;
    }
  }

// NEW: Test mode - cycles through outputs
void processTestMode() {
  static unsigned long lastTestUpdate = 0;
  static uint8_t testStep = 0;
  
  if (millis() - lastTestUpdate >= 2000) {  // Change every 2 seconds
    lastTestUpdate = millis();
    
    switch (testStep) {
      case 0:
        digitalWrite(RELAY1_PIN, HIGH);
        digitalWrite(RELAY2_PIN, LOW);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, LOW);
        Serial.println("🧪 Test Mode: Step 1 - Relay1 ON, Green LED");
        break;
      case 1:
        digitalWrite(RELAY1_PIN, LOW);
        digitalWrite(RELAY2_PIN, HIGH);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_RED, LOW);
        Serial.println("🧪 Test Mode: Step 2 - Relay2 ON, Yellow LED");
        break;
      case 2:
        digitalWrite(RELAY1_PIN, HIGH);
        digitalWrite(RELAY2_PIN, HIGH);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, HIGH);
        Serial.println("🧪 Test Mode: Step 3 - Both Relays ON, Red LED");
        break;
      case 3:
        digitalWrite(RELAY1_PIN, LOW);
        digitalWrite(RELAY2_PIN, LOW);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_RED, HIGH);
        Serial.println("🧪 Test Mode: Step 4 - All OFF, All LEDs ON");
        break;
    }
    
    testStep = (testStep + 1) % 4;
  }
}

void checkCommunication() {

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    if (!(moduleStatus & STATUS_COMM_ERROR)) {
      moduleStatus |= STATUS_COMM_ERROR;
      Serial.println(" WiFi Disconnected");
      digitalWrite(LED_RED, HIGH);
    }
    
    // Try to reconnect
    if (millis() % 10000 == 0) {
      Serial.println(" Attempting WiFi reconnection...");
      WiFi.reconnect();
    }
  } else {
    moduleStatus &= ~STATUS_COMM_ERROR;
  }

  static unsigned long lastModbusActivity = 0;
  
  // Detect Modbus activity
  /*if (mb.slave() == SLAVE_ID) {
    lastModbusActivity = millis();
    moduleStatus &= ~STATUS_COMM_ERROR;
  }*/
  
  // Check timeout
  /*if (millis() - lastModbusActivity > COMM_TIMEOUT) {
    if (!(moduleStatus & STATUS_COMM_ERROR)) {
      moduleStatus |= STATUS_COMM_ERROR;
      Serial.println(" Communication timeout with PLC");
      
      // Failsafe: Turn off all outputs
      digitalWrite(RELAY1_PIN, LOW);
      digitalWrite(RELAY2_PIN, LOW);
      digitalWrite(LED_RED, HIGH);
    }
  }*/
}

void updateModuleStatus() {
  uint16_t mode = mb.Hreg(HR_MODULE_MODE);
  
  // Update mode status bits
  if (mode == MODE_MANUAL) {
    moduleStatus |= STATUS_MANUAL_MODE;
    moduleStatus &= ~STATUS_TEST_MODE;
  } else if (mode == MODE_TEST) {
    moduleStatus |= STATUS_TEST_MODE;
    moduleStatus &= ~STATUS_MANUAL_MODE;
  } else {
    moduleStatus &= ~(STATUS_MANUAL_MODE | STATUS_TEST_MODE);
  }
  
  // Check temperature alarm (only in AUTO mode)
  if (mode == MODE_AUTO) {
    float temp = inputRegisters[IR_TEMP] / 10.0;
    float tempSetpoint = mb.Hreg(HR_TEMP_SETPOINT) / 10.0;
    float hysteresis = mb.Hreg(HR_HYS_SETPOINT) / 10.0;
    
    // Alarm if temperature exceeds setpoint + 2x hysteresis
    if (temp > (tempSetpoint + 2 * hysteresis)) {
      moduleStatus |= STATUS_ALARM_ACTIVE;
    } else {
      // Check if alarm acknowledged
      if (mb.Hreg(HR_ALARM_ACK)) {
        moduleStatus &= ~STATUS_ALARM_ACTIVE;
        mb.Hreg(HR_ALARM_ACK, 0); // Clear ack bit
        Serial.println("✓ Alarm Acknowledged");
      }
    }
  }
  
  // Update module status register
  inputRegisters[IR_MODULE_STATUS] = moduleStatus;
  mb.Ireg(IR_MODULE_STATUS, moduleStatus);
  
  // Debug output every 5 seconds
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 2000) {
    lastDebug = millis();
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println(" I/O Module Status Report");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("Temperature:  %.1f°C\n", inputRegisters[IR_TEMP] / 10.0);
    Serial.printf("Humidity:     %.1f%%\n", inputRegisters[IR_HUMIDITY] / 10.0);
    Serial.printf("Oil Level:    %d cm\n", inputRegisters[IR_OIL_LEVEL]);
    Serial.printf("Relay 1:      %s\n", digitalRead(RELAY1_PIN));
    Serial.printf("Relay 2:      %s\n", digitalRead(RELAY2_PIN));
    Serial.printf("Mode:         %s\n", mode == MODE_AUTO ? "AUTO" : mode == MODE_MANUAL ? "MANUAL" : "TEST");
    Serial.printf("WiFi Signal:  %d dBm\n", WiFi.RSSI());
    Serial.printf("Module Status: 0x%04X\n", moduleStatus);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  }

}
