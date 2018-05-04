/*
    Single reading mode.
    Takes a reading in intervals and sends the data to the server once for every reading.
*/

// Includes
#include "gps_math.h"
#include "Adafruit_GPS.h"
#include "Particle.h"
#include "math.h"
#include "ctype.h"
#include "StringSplitter.h"

// Variables
int led_pin = D7;
int gps_pin = D6;
int command_mode_button = D4;

// I2C addresses
#define rtd_address 102
#define ph_address 99
#define do_address 97
#define ec_address 100

// Server
TCPClient client;
byte server[] = { 34, 210, 238, 35 };       // AWS
String ip = "34.210.238.35";                // AWS
int port = 1221;
bool SEND_DATA_TO_SERVER = false;
bool LOG_DATA_TO_SERIAL = true;

// Command mode variables
char computerdata[20];              // Data/command incoming from a computer
byte received_from_computer = 0;    // Length of incoming data
byte serial_event = 0;              // A flag to indicate if data was received from the computer
byte code = 0;                      // Holds the I2C response code. 
char RTD_data[20];                  // Incoming data from the RTD circuit. 
byte in_char = 0;                   // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
byte i = 0;                         // Counter used for RTD_data array. 
int time_ = 600;                    // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 
float temp_float;                   // Float var used to hold the float value of the RTD. 

int power_timer = 0;

FuelGauge fuel;

// GPS variables
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
int lastSecond = 0;
bool ledState = false;
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
unsigned long lastPublish = 0;
time_t lastIdleCheckin = 0;

// Control variables
bool COMMAND_MODE = 0;
String COMMAND = "r";
String COMMAND_TARGET = "rtd";
int DELAY_BETWEEN_READINGS = 60 * 60 * 1000;
int HOW_LONG_SHOULD_WE_SLEEP = 6 * 60 * 60;
int MAX_IDLE_CHECKIN_DELAY = HOW_LONG_SHOULD_WE_SLEEP - 60;
int WAIT_FOR_SAT_FIX = 0;
int DELAY_BETWEEN_COMMAND_MODE_ITERATIONS = 5 * 1000;
int DELAY_BETWEEN_CONTROL_VARS_UPDATE = 5 * 1000;
int DELAY_WHILE_WAIT_FOR_SAT_FIX = 6 * 1000;
int TURN_OFF_FOR = 0;
bool CELLULAR_ACTIVE = 1; 

// Device setup
void setup(){
    lastPublish = 0;
    
    pinMode(gps_pin, OUTPUT);                               // Electron asset tracker shield needs this to enable the power to the gps module.
    digitalWrite(gps_pin, LOW);                             // Turn GPS on
    
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    
    delay(250);                                             // Wait for the GPS to wake up
    
    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);
    
    GPS.sendCommand("$PMTK101*32");                         // GPS hot restart
    delay(250);
    
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);          // Request everything
    delay(250);
    
    GPS.sendCommand(PGCMD_NOANTENNA);                       // turn off antenna updates
    delay(250);
    
    pinMode(command_mode_button, INPUT);
    
    Serial.begin(9600);                                     // Enable serial port.  
    Wire.begin();                                           // Enable I2C.
    
    print("Setup complete");
}

// Device loop and timing variables
long last_sensors_probed_at = 0;
long last_command_to_sensors_at = 0;
long last_control_vars_updated_at = 0;

void loop(){
    long now = millis();                                    // Current timestamp
    
    // Update control variables
    if(((now - last_control_vars_updated_at) > DELAY_BETWEEN_CONTROL_VARS_UPDATE) || last_control_vars_updated_at == 0){   
        last_control_vars_updated_at = now;
        getControlVariables("all");
    }
    
    // Listen for turn off command from the server
    if(TURN_OFF_FOR > 0)
        print("Server requested the device to sleep. Sleeping for " + String(TURN_OFF_FOR) + " milliseconds.");
        
    while(TURN_OFF_FOR > 0){
        long now = millis();
    
        if(CELLULAR_ACTIVE == 1){
            Cellular.off();
            CELLULAR_ACTIVE = 0;
        }            
        TURN_OFF_FOR -= 1000;
        delay(1000);
    }
    if(CELLULAR_ACTIVE == 0){
        Cellular.on();
        CELLULAR_ACTIVE = 1;
        
        print("Waking up from a server-requested sleep.");
    }
    
    // Battery saver
    if(!CELLULAR_ACTIVE && false){
        if(power_timer  >= 6 * 60 * 60 * 1000){             // Sleep for 6hrs
            toggleCellular();                               // Turn on cellular module
            power_timer = 0;
        }
        else{
            power_timer += 1000;
        }
        
        delay(1000);
    }
    else{
        // Check if the device needs to be put in i2c command mode
        if(digitalRead(command_mode_button) == HIGH)
            COMMAND_MODE = false;
        else
            COMMAND_MODE = true;
        
        // Check if the device needs to be in command mode, in command mode, the device waits for the serial monitor for commands.
        if(COMMAND_MODE && ((now - last_command_to_sensors_at) > DELAY_BETWEEN_COMMAND_MODE_ITERATIONS) || last_command_to_sensors_at == 0){
            last_command_to_sensors_at = now;               // Update last_sensors_probed_at
            
            getData("rtd", true);                           // Put RTD sensor to command mode
            getData("ph", true);                         // Put pH sensor to command mode
            getData("do", true);                         // Put DO sensor to command mode
            getData("ec", true);                            // Put EC sensor to command mode
        }
        else if(((now - last_sensors_probed_at) > DELAY_BETWEEN_READINGS) || last_sensors_probed_at == 0){
            last_sensors_probed_at = now;                   // Update last_sensors_probed_at
            
            // Get data from the sensors and send it to the server
            getSensorData();
        }
    }
    
    delay(4000);
}

void getSensorData(){
    // Collect data from the sensors
    float rtd_float = getData("rtd", false);
    float ph_float = 0;
    float do_float = 0;
    float ec_float = getData("ec", false);
    
    // Check GPS
    checkGPS();
    delay(10);
    String gpsData = getGPSData();
    
    // Wait for sat fix 
    if(strcmp(gpsData, "0.000000,-0.000000") == 0 && WAIT_FOR_SAT_FIX){
        print("Waiting for a SAT fix. Hold on.");
        
        long last_gps_fix_check_at = 0;
        while(strcmp(gpsData, "0.000000,-0.000000") == 0 && WAIT_FOR_SAT_FIX){
            long now = millis();
            
            if(((now - last_gps_fix_check_at) > DELAY_WHILE_WAIT_FOR_SAT_FIX) || last_gps_fix_check_at == 0){
                blink(led_pin, 200, 200, 2);
                checkGPS();
                delay(10);
                gpsData = getGPSData();
                last_gps_fix_check_at = millis();
            }
            
            // // Update control variables
            // if(((now - last_control_vars_updated_at) > DELAY_BETWEEN_CONTROL_VARS_UPDATE) || last_control_vars_updated_at == 0){  
            //     print("Here 3"); 
            //     last_control_vars_updated_at = now;
            //     getControlVariables("all");
            // }
        }
    }
    
    // Send the data to the server
    String data_str = "{\"do\":" + String(do_float) + ",\"ec\":" + String(ec_float) + ",\"ph\":" + String(ph_float) + ",\"rtd\":" + String(rtd_float) + ",\"location\":\"" + gpsData + "\"" + ",\"batt-volt\":" + fuel.getVCell() + ",\"batt-level\":" + fuel.getSoC() + ",\"utc-timestamp\":\"" + getTime("utc") + "\"}";
    sendData("all", data_str);
}

// Toggle cellular on the device
void toggleCellular(){
    if(CELLULAR_ACTIVE)
        Cellular.off();
    else
        Cellular.on();
    CELLULAR_ACTIVE = !CELLULAR_ACTIVE;
}

// Collect sensor data
float getData(String sensor_type, bool command_mode){
    int sensor_address;
    char sensor_data[20];
    String cmd;
    
    // Sensor specific actions
    if (strcmp(sensor_type, "rtd") == 0) {
        sensor_address = rtd_address;
    }
    else if (strcmp(sensor_type, "ph") == 0) {
        sensor_address = ph_address;
    }
    else if (strcmp(sensor_type, "do") == 0) {
        sensor_address = do_address;
    }
    else if (strcmp(sensor_type, "ec") == 0) {
        sensor_address = ec_address;
    }

    if(COMMAND_MODE && serial_event == true){
        computerdata[0] = tolower(computerdata[0]);                               
        cmd = computerdata;
    }
    else if(COMMAND_MODE && serial_event == false){
        cmd = COMMAND;
    }
    else if(!COMMAND_MODE){
        cmd = "r";
    }                               
    
    // Set appropriate delay
    if(cmd[0] == 'c'|| cmd[0] == 'r')
        time_ = 600;                                            // If a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.  
    else 
        time_ = 300;                                            // If any other command has been sent we wait only 300ms.
    
    // Send command to the sensor
    Wire.beginTransmission(sensor_address);                     // Call the circuit by its ID number.  
    Wire.write(cmd);                                            // Transmit the command that was sent through the serial port.
    Wire.endTransmission();                                     // End the I2C data transmission. 
    
    if (strcmp(cmd, "sleep") != 0) {                            // If the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
        delay(time_);                                           // Wait the correct amount of time for the circuit to complete its instruction. 
        
        Wire.requestFrom(sensor_address, 20, 1);                // Call the circuit and request 20 bytes (this may be more than we need)
        code = Wire.read();                                     // The first byte is the response code, we read this separately.  
        
        // Read the response of the command sent above
        while(Wire.available()){                                // While there are more bytes to receive.  
            in_char = Wire.read();                              // Read the next byte.
            sensor_data[i] = in_char;                           
            i += 1;                                           
            if(in_char == 0){                                   // If the byte is a null command. 
                i = 0;                                          // Reset the counter i to 0.
                Wire.endTransmission();                         // End the I2C data transmission.
                break;
            }
        }
            
        if(!COMMAND_MODE && false){
            Particle.publish(sensor_type, String(sensor_data));
            Serial.println("\n" + sensor_type + ": " + String(sensor_data));
        }
        else if(serial_event)
            Serial.println(cmd + ": " + String(sensor_data));
    }
    
    delay(500);                                                // Wait a moment before reading the next sensor
    
    if(!COMMAND_MODE)
        return atof(sensor_data);
    else{
        serial_event = false;
        return 0.0;
    }

}

// Send data to the server
void sendData(String sensor_type, String data){
    if(SEND_DATA_TO_SERVER){
        // Connect to the server
        connect();
        
        // Send data to the server
        blink(led_pin, 200, 200, 4);
        client.println("GET /api/" + sensor_type + "/" + data + " HTTP/1.0");
        client.println("Host: " + ip);
        client.println("Content-Length: 0");
        client.println();
        
        // Disconnect from the server
        disconnect();
    }
}

// Get time from the server
String getTime(String type){
    TCPClient client;
    
    // Connect and send data to the server
    if (client.connect(ip, port)){
        client.println("GET /time/" + type + " HTTP/1.0");
        client.println("Host: " + ip);
        client.println("Content-Length: 0");
        client.println();
        
        String response = getResponse(client);
        
        if(strcmp(response, "20") == 0){
            toggleCellular();
        }

        // Disconnect from the server
        disconnect();
        
        return response;
    }
}

// Get control variables from the server
void getControlVariables(String type){
    print("Updating control variables");
    
    TCPClient client;
    
    // Connect and send data to the server
    if (client.connect(ip, port)){
        beacon(led_pin, 2);
        client.println("GET /control/" + type + " HTTP/1.0");
        client.println("Host: " + ip);
        client.println("Content-Length: 0");
        client.println();
        
        String response = getResponse(client);

        // Disconnect from the server
        disconnect();
        
        // Update control variables
        updateControlVariables(response);
    }
}

void updateControlVariables(String response){
    print("Control variables updated.");
    
    response.replace("{", "").replace("}", "");
    
    StringSplitter *splitter = new StringSplitter(response, ',', 20);
    int itemCount = splitter->getItemCount();
    
    HOW_LONG_SHOULD_WE_SLEEP = ((new StringSplitter(splitter->getItemAtIndex(0), ':', 2))->getItemAtIndex(1)).toInt();
    WAIT_FOR_SAT_FIX = ((new StringSplitter(splitter->getItemAtIndex(1), ':', 2))->getItemAtIndex(1)).toInt();
    DELAY_BETWEEN_READINGS = ((new StringSplitter(splitter->getItemAtIndex(2), ':', 2))->getItemAtIndex(1)).toInt();
    DELAY_BETWEEN_COMMAND_MODE_ITERATIONS = ((new StringSplitter(splitter->getItemAtIndex(3), ':', 2))->getItemAtIndex(1)).toInt();
    COMMAND_MODE = ((new StringSplitter(splitter->getItemAtIndex(4), ':', 2))->getItemAtIndex(1)).toInt();
    COMMAND_TARGET = ((new StringSplitter(splitter->getItemAtIndex(5), ':', 2))->getItemAtIndex(1));
    COMMAND = ((new StringSplitter(splitter->getItemAtIndex(6), ':', 2))->getItemAtIndex(1));
    TURN_OFF_FOR = ((new StringSplitter(splitter->getItemAtIndex(7), ':', 2))->getItemAtIndex(1)).toInt();
    SEND_DATA_TO_SERVER = ((new StringSplitter(splitter->getItemAtIndex(8), ':', 2))->getItemAtIndex(1)).toInt();
    LOG_DATA_TO_SERIAL = ((new StringSplitter(splitter->getItemAtIndex(9), ':', 2))->getItemAtIndex(1)).toInt();
}

String getResponse(TCPClient client){
    String response = "";
    // Wait until the response is avilable
    while(!client.available());
    
    // When the response is available do stuff
    while(client.available()){
        char c = client.read();
        response += c;
        
        // Remove HTTP header
        if(response.endsWith("\n"))
            response = "";
    }
    return response;
}

void publish(char key[], char value[]){
    Particle.publish(String(key), String(value));
}

bool connect(){    
    if (client.connect(ip, port)){
        // beacon(led, 4);
        return true;
    }
    else{
        blink(led_pin, 2500, 500, 4);
        Particle.publish("connected", "False");
    }
}

void disconnect(){
    client.stop();
}

void wait(int time){
    delay(time);
    beacon(led_pin, time/1000);
}

// Blink led function
void blink(int led, int on_time, int off_time, int repeatations){
    for(int i = 0; i < repeatations; i++){
        digitalWrite(led, HIGH);
        delay(on_time);
        digitalWrite(led, LOW);
        delay(off_time);
    }
}

// Beacon led function
void beacon(int led, int repeatations){
    blink(led, 50, 1950, repeatations);
}

// Interrupt for when computer sends data
void serialEvent(){                                                        
    received_from_computer=Serial.readBytesUntil(13,computerdata,20);      
    computerdata[received_from_computer]=0;                              // Stop the buffer from transmitting leftovers or garbage.
    serial_event = true;                                                 // Set the serial event flag.
}

void print(String message){
    if(LOG_DATA_TO_SERIAL)
        Serial.println(message);
}

void getBatteryLevel(){
    float voltage, level;
    voltage = fuel.getVCell();
    level = fuel.getSoC();
    
    String output = "{\"battery-voltage\": \"" + String(voltage) + "\", \"battery-level\": \"" + String(level) + "\"}";
    Particle.publish("battery", output);
}

void checkGPS() {
    while (mySerial.available()) {
        char c = GPS.read();

        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
    }
}

String getGPSData() {
    String gps_line =
          "{\"lat\":"    + String(convertDegMinToDecDeg(GPS.latitude))
        + ",\"lon\":-"   + String(convertDegMinToDecDeg(GPS.longitude))
        + ",\"a\":"     + String(GPS.altitude)
        + ",\"q\":"     + String(GPS.fixquality)
        + ",\"spd\":"   + String(GPS.speed)
        + ",\"s\": "  + String(GPS.satellites)
        + ",\"vcc\":"   + String(fuel.getVCell())
        + ",\"soc\":"   + String(fuel.getSoC())
        + "}";
    
    return String(convertDegMinToDecDeg(GPS.latitude)) + "," + "-" + String(convertDegMinToDecDeg(GPS.longitude));
}

char* s2c(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}