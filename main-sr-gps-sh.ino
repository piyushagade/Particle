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
int DELAY_BETWEEN_READINGS = 30 * 60 * 1000;
int HOW_LONG_SHOULD_WE_SLEEP = 6 * 60 * 60;
int MAX_IDLE_CHECKIN_DELAY = HOW_LONG_SHOULD_WE_SLEEP - 60;
int WAIT_FOR_SAT_FIX = 0;
int DELAY_BETWEEN_COMMAND_MODE_ITERATIONS = 5 * 1000;
int DELAY_BETWEEN_CONTROL_VARS_UPDATE = 10 * 60 * 1000;     // Add this to control.json?
int DELAY_BETWEEN_HIBERNATION_HOUR_CHECKS = 20 * 60 * 1000;     // Add this to control.json?
int DELAY_WHILE_WAIT_FOR_SAT_FIX = 6 * 1000;
int SCHEDULED_HIBERNATION = 1;
int TURN_OFF_FOR = 0;
int REBOOT_IN_SAFE_MODE = 0;
int SEND_DATA_TO_SERVER = 1;
int LOG_DATA_TO_SERIAL = 1;
int LOG_DATA_TO_CLOUD = 0;
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
    
    beacon(led_pin, 5);
    
    delay(4000);
    print("Setup complete", true);
    print("Waking up with battery percentage at: " + String(fuel.getSoC()), true);
}

// Device loop and timing variables
long last_sensors_probed_at = 0;
long last_command_to_sensors_at = 0;
long last_control_vars_updated_at = 0;
long last_hibernation_hour_checked_at = 0;
int hour;

void loop(){    
    // Update control variables
    if(((millis() - last_control_vars_updated_at) > DELAY_BETWEEN_CONTROL_VARS_UPDATE) || last_control_vars_updated_at == 0){   
        last_control_vars_updated_at = millis();
        getControlVariables("all?reset=true");
<<<<<<< HEAD
    }
    
    // Reboot in safe mode
    if(REBOOT_IN_SAFE_MODE){
        print("Server requested the device to reboot in safe mode. Rebooting in 4000 milliseconds.", true);
        delay(4000);
        System.enterSafeMode();
    }
    
    
    Serial.println(TURN_OFF_FOR > 0, true);
    delay(10000);
    
    // // Listen for turn off command from the server / forced sleep
    // if(TURN_OFF_FOR > 0){
    //     print("Server requested the device to sleep. Sleeping for " + String(TURN_OFF_FOR) + " milliseconds.", true);
    //     delay(1000);
    //     System.sleep(SLEEP_MODE_DEEP, (TURN_OFF_FOR / 1000) + 1);
    // }
    
=======
    }
    
    // Reboot in safe mode
    if(REBOOT_IN_SAFE_MODE){
        print("Server requested the device to reboot in safe mode. Rebooting in 4000 milliseconds.", true);
        delay(4000);
        System.enterSafeMode();
    }
    
    // Listen for turn off command from the server / forced sleep
    if(TURN_OFF_FOR > 0){
        print("Server requested the device to sleep. Sleeping for " + String(TURN_OFF_FOR) + " milliseconds.", true);
        delay(1000);
        System.sleep(SLEEP_MODE_DEEP, (TURN_OFF_FOR / 1000) + 1);
    }
    
>>>>>>> 62c2c11a2b0536d1d9e74fb64a38a849a92912e7
    // Scheduled hibernation
    if(((millis() - last_hibernation_hour_checked_at) > DELAY_BETWEEN_HIBERNATION_HOUR_CHECKS) || last_hibernation_hour_checked_at == 0){
        last_hibernation_hour_checked_at = millis();
        hour = getTime("hour").toInt();
    }
    
    // Compute hours limits for hibernation
<<<<<<< HEAD
    int turn_off_hour = -1 + 4;
    int turn_on_hour = 11 + 4;
    
    if((hour >= turn_off_hour && hour <= turn_on_hour) & SCHEDULED_HIBERNATION && false){
=======
    int turn_off_hour = -1 + 4;                                  // 6 AM
    int turn_on_hour = 11 + 4;                                  // 6 PM
    if((hour >= turn_off_hour && hour <= turn_on_hour) & SCHEDULED_HIBERNATION){
>>>>>>> 62c2c11a2b0536d1d9e74fb64a38a849a92912e7
        print("Sleeping for " + String(turn_on_hour - turn_off_hour) + " hours.", true);
        delay(10000);
        System.sleep(SLEEP_MODE_DEEP, (turn_on_hour - turn_off_hour) * 3600);
    }
    else{
        // Check if the device needs to be put in i2c command mode
        if(digitalRead(command_mode_button) == HIGH)
            COMMAND_MODE = false;
        else
            COMMAND_MODE = true;
        
        // Check if the device needs to be in command mode, in command mode, the device waits for the serial monitor for commands.
        if(COMMAND_MODE && ((millis() - last_command_to_sensors_at) > DELAY_BETWEEN_COMMAND_MODE_ITERATIONS) || last_command_to_sensors_at == 0){
            last_command_to_sensors_at = millis();          // Update last_sensors_probed_at
            
            getData("rtd", true);                           // Put RTD sensor to command mode
            getData("ph", true);                            // Put pH sensor to command mode
            getData("do", true);                            // Put DO sensor to command mode
            getData("ec", true);                            // Put EC sensor to command mode
        }
        else if(((millis() - last_sensors_probed_at) > DELAY_BETWEEN_READINGS) || last_sensors_probed_at == 0){
            last_sensors_probed_at = millis();              // Update last_sensors_probed_at
            
            // Get data from the sensors and send it to the server
            getSensorData();
        }
    }
    
    delay(20000);
}

void getSensorData(){
    // Check GPS
    checkGPS();
    delay(10);
    String gpsData = getGPSData();
    
    // Wait for sat fix 
    if(strcmp(gpsData, "0.000000,-0.000000") == 0 && WAIT_FOR_SAT_FIX){
        print("Waiting for a SAT fix. Hold on.", true);
        
        long last_gps_fix_check_at = 0;
        while(strcmp(gpsData, "0.000000,-0.000000") == 0 && WAIT_FOR_SAT_FIX){
            long now = millis();
            
            if(((now - last_gps_fix_check_at) > DELAY_WHILE_WAIT_FOR_SAT_FIX) || last_gps_fix_check_at == 0){
                last_gps_fix_check_at = millis();
                blink(led_pin, 200, 200, 2);
                checkGPS();
                delay(10);
                gpsData = getGPSData();
            }
            
            // Update control variables
            if(((now - last_control_vars_updated_at) > DELAY_BETWEEN_CONTROL_VARS_UPDATE) || last_control_vars_updated_at == 0){  
                last_control_vars_updated_at = now;
                getControlVariables("all");
            }
        }
    }
    
    // Collect data from the sensors
    float rtd_float = getData("rtd", false);
    float ph_float = 0;
    float do_float = 0;
    float ec_float = getData("ec", false);
    
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
            print("\n" + sensor_type + ": " + String(sensor_data), true);
        }
        else if(serial_event)
            print(cmd + ": " + String(sensor_data), true);
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
        disconnect(client);
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

        // Disconnect from the server
        disconnect(client);
        
        return response;
    }
}

// Get control variables from the server
void getControlVariables(String type){
    print("Updating control variables", false);
    
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
        disconnect(client);
        
        // Update control variables
        updateControlVariables(response);
    }
}

// Update the control variables
void updateControlVariables(String response){
    print(" -> Done", true);
    HOW_LONG_SHOULD_WE_SLEEP = (parseControlVariables(response, "HOW_LONG_SHOULD_WE_SLEEP").toInt());
    WAIT_FOR_SAT_FIX = (parseControlVariables(response, "WAIT_FOR_SAT_FIX").toInt());
    DELAY_BETWEEN_READINGS = (parseControlVariables(response, "DELAY_BETWEEN_READINGS").toInt());
    DELAY_BETWEEN_COMMAND_MODE_ITERATIONS = (parseControlVariables(response, "DELAY_BETWEEN_COMMAND_MODE_ITERATIONS").toInt());
    COMMAND_MODE = (parseControlVariables(response, "COMMAND_MODE").toInt());
    COMMAND_TARGET = (parseControlVariables(response, "COMMAND_TARGET"));
    COMMAND = (parseControlVariables(response, "COMMAND"));
    TURN_OFF_FOR = (parseControlVariables(response, "TURN_OFF_FOR").toInt());
    SEND_DATA_TO_SERVER = (parseControlVariables(response, "SEND_DATA_TO_SERVER").toInt());
    LOG_DATA_TO_SERIAL = (parseControlVariables(response, "LOG_DATA_TO_SERIAL").toInt());
    LOG_DATA_TO_CLOUD = (parseControlVariables(response, "LOG_DATA_TO_CLOUD").toInt());
    REBOOT_IN_SAFE_MODE = (parseControlVariables(response, "REBOOT_IN_SAFE_MODE").toInt());
    SCHEDULED_HIBERNATION = (parseControlVariables(response, "SCHEDULED_HIBERNATION").toInt());
}

// Parse the control response
String parseControlVariables(String response, String key){
    String pair = "";
    String result, temp_key, value;
    
    for(int i = 0; i < response.length(); i++){
        if(response.charAt(i) == '['){
            // Do nothing
        }
        else{
            if(response.charAt(i) == ',')
                pair = "";
            else
                pair += String(response.charAt(i));
            
            // Check if the pair is what the user wants
            // Part A: Construct the temp_key
            bool flag_temp_key_generation_complete = false;
            temp_key = "";
            for(int j = 0; j < pair.length(); j++){
                if(pair.charAt(j) == ':')   {
                    flag_temp_key_generation_complete = true;
                }
                else{
                    if(!flag_temp_key_generation_complete)
                        temp_key += String(pair.charAt(j));
                }
            }
            breakB:
            
            // Part B: Comapre the keys if the temp key was generated completely
            if(key == temp_key && flag_temp_key_generation_complete){
                // Wait until the pair is completely generated
                value = "";
                for(int j = pair.indexOf(':') + 1; j < pair.length(); j++)
                    value += pair.charAt(j);
                    
                    // Look for the end
                    if(response.charAt(i + 1) == ',' || response.charAt(i + 1) == ']') 
                        return value;
            }
        }
    }

    // If nothing found
    return "";
}

// Get response from the server
String getResponse(TCPClient client){
    String response = "";
    // Wait until the response is avilable
    while(!client.available());
    
    // When the response is available do stuff
    while(client.available()){
        char c = client.read();
        response += c;
        
        // Remove HTTP header / extract the body
        if(response.endsWith("\n"))
            response = "";
    }
    return response;
}

// Publish to particle
void publish(String key, String value){
    Particle.publish(String(key), String(value));
}

// Connect to the server
bool connect(){    
    if (client.connect(ip, port)){
        // beacon(led, 4);
        return true;
    }
    else{
        blink(led_pin, 2500, 500, 4);
        Particle.publish("connected", "false");
    }
}

// Disconnect the server
void disconnect(TCPClient client){
    client.stop();
}

// Wait with beacon
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
    received_from_computer = Serial.readBytesUntil(13, computerdata, 20);      
    computerdata[received_from_computer] = 0;                              // Stop the buffer from transmitting leftovers or garbage.
    serial_event = true;                                                 // Set the serial event flag.
}

// Log to serial/cloud
void print(String message, bool new_line){
    if(LOG_DATA_TO_SERIAL)
        if(new_line)
            Serial.println(message);
        else
            Serial.print(message);
    
    if(LOG_DATA_TO_CLOUD)
        publish("log", message);
}

// Get battery stat
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
