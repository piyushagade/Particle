// Includes
#include "Particle.h"

// Variables
int led = D7;

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

char computerdata[20];           // Data incoming from a computer   
byte received_from_computer=0;   // Length of incoming data
byte serial_event=0;             // A flag to indicate if data was received from the computer
byte code=0;                     // Holds the I2C response code. 
char RTD_data[20];               // Incoming data from the RTD circuit. 
byte in_char=0;                  // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
byte i=0;                        // Counter used for RTD_data array. 
int time_=600;                   // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 
float temp_float;                // Float var used to hold the float value of the RTD. 
bool cellular_active = true; 

// Device setup
void setup(){
    pinMode(led, OUTPUT);         // Set led pin as output
    Serial.begin(9600);           // Enable serial port.  
    Wire.begin();                 // Enable I2C.
}

// Device loop
void loop(){
     // Collect data
    float rtd_float = getData("rtd");
    float ph_float = getData("ph");
    float do_float = getData("do");
    float ec_float = getData("ec");

    // Send data
    sendData("rtd", String(rtd_float));
    sendData("ph", String(ph_float));
    sendData("do", String(do_float));
    sendData("ec", String(ec_float));

    // Wait before iterating and taking another reading from sensors
    delay(30000);
}

// Toggle cellular on the device
void togglePower(){
    if(cellular_active)
        Cellular.off();
    else
        Cellular.on();
    cellular_active = !cellular_active;
}

// Collect sensor data
float getData(String sensor_type){
    int sensor_address;
    char sensor_data[20];
    
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
        
    // Command to be send to the device
    String cmd = "r";
    
    if(cmd[0] == 'c'|| cmd[0] == 'r')
        time_ = 600;                                          // If a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.  
    else 
        time_ = 300;                                          // If any other command has been sent we wait only 300ms.
    
    // Send command to the sensor
    Wire.beginTransmission(sensor_address);                    // Call the circuit by its ID number.  
    Wire.write(cmd);                                        // Transmit the command that was sent through the serial port.
    Wire.endTransmission();                                 // End the I2C data transmission. 
    
    if (strcmp(computerdata, "sleep") != 0) {               // If the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
        delay(time_);                                       // Wait the correct amount of time for the circuit to complete its instruction. 
        
        Wire.requestFrom(sensor_address, 20, 1);                 // Call the circuit and request 20 bytes (this may be more than we need)
        code = Wire.read();                                   // The first byte is the response code, we read this separately.  
        
        // Read the response of the command sent above
        while(Wire.available()){                            // While there are more bytes to receive.  
            in_char = Wire.read();                          // Read the next byte.
            sensor_data[i] = in_char;                           
            i += 1;                                           
            if(in_char == 0){                                 // If the byte is a null command. 
                i = 0;                                        // Reset the counter i to 0.
                Wire.endTransmission();                     // End the I2C data transmission.
                break;
            }
        }
            
        Particle.publish(sensor_type, String(sensor_data));
    }
    return atof(sensor_data);

    delay(1000);
}

// Send data to the server
void sendData(String sensor_type, String endpoint){
    // Connect to the server
    connect();
    
    // Send data to the server
    blink(led, 1000, 1000, 1);
    client.println("GET /insert/" + sensor_type + "/" + endpoint + " HTTP/1.0");
    client.println("Host: " + ip);
    client.println("Content-Length: 0");
    client.println();
    
    // Disconnect from the server
    disconnect();
}

void publish(char key[], char value[]){
    Particle.publish(String(key), String(value));
}

void connect(){    
    if (client.connect(ip, port)){
        // beacon(led, 4);
        Particle.publish("connected", "True");
    }
    else{
        // blink(led, 2500, 500, 4);
        Particle.publish("connected", "False");
    }
}

void disconnect(){
    client.stop();
}

void wait(int time){
    delay(time);
    beacon(led, time/1000);
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
    for(int i = 0; i < repeatations; i++){
        digitalWrite(led, HIGH);
        delay(50);
        digitalWrite(led, LOW);
        delay(1950);
    }
}

// Interrupt for when computer sends data
void serialEvent(){                                                        
    received_from_computer=Serial.readBytesUntil(13,computerdata,20);      
    computerdata[received_from_computer]=0;                              // Stop the buffer from transmitting leftovers or garbage.
    serial_event = true;                                                 // Set the serial event flag.
}