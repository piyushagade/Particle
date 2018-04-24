// Includes
#include "Particle.h"

// Variables
int led = D7;

// Sensors
#define rtd_address 102

// Server
TCPClient client;
byte server[] = { 34, 210, 238, 35 }; // AWS
int port = 1221;
bool active = true;

char computerdata[20];           // Data incoming from a computer   
byte received_from_computer=0;   // Length of incoming data
byte serial_event=0;             // A flag to indicate if data was received from the computer
byte code=0;                     // Holds the I2C response code. 
char RTD_data[20];               // Incoming data from the RTD circuit. 
byte in_char=0;                  // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
byte i=0;                        // Counter used for RTD_data array. 
int time_=600;                   // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 
float tmp_float;                 // Float var used to hold the float value of the RTD. 

// Device setup
void setup(){
    pinMode(led, OUTPUT);         // Set led pin as output
    Serial.begin(9600);           // Enable serial port.  
    Wire.begin();                 // Enable I2C.
}

// Device loop
void loop(){
    // Collect data
    getData();

    // Send data
    sendData("GET /insert/rtd/81.01 HTTP/1.0");

    // Wait before iterating
    delay(30000);
}

// Toggle cellular on the device
void togglePower(){
    if(active)
        Cellular.off();
    else
        Cellular.on();
    active = !active;
}

// Collect sensor data
void getData(){
    if (true) {
        computerdata[0] = tolower(computerdata[0]);
        if(computerdata[0]=='c'||computerdata[0]=='r')
            time_=600;                                          // If a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.  
        else 
            time_=300;                                          // If any other command has been sent we wait only 300ms.
       
    
        Wire.beginTransmission(rtd_address);                    // Call the circuit by its ID number.  
        Wire.write(computerdata);                               // Transmit the command that was sent through the serial port.
        Wire.endTransmission();                                 // End the I2C data transmission. 
        
    
        if (strcmp(computerdata, "sleep") != 0) {               // If the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
            delay(time_);                                       // Wait the correct amount of time for the circuit to complete its instruction. 
            
            Wire.requestFrom(rtd_address,20,1);                 // Call the circuit and request 20 bytes (this may be more than we need)
            code=Wire.read();                                   // The first byte is the response code, we read this separately.  
            
            switch (code){                                      // Switch case based on what the response code is.  
                case 1:                                         // Successful command.
                    Serial.println("Success");  
                    break;                        
                
                case 2:                                         // Command failed
                    Serial.println("Failed");    
                    break;                       
                
                case 254:                                       // Command has not yet finished calculations.
                    Serial.println("Pending");
                    break;                         
                
                case 255:                                       // There is no more data to send.
                    Serial.println("No Data");   
                    break;                         
            }
            
            // Read the response of the command sent above
            while(Wire.available()){                            // While there are more bytes to receive.  
                in_char = Wire.read();                          // Read the next byte.
                RTD_data[i]= in_char;                           
                i+=1;                                           
                if(in_char==0){                                 // If the byte is a null command. 
                    i=0;                                        // Reset the counter i to 0.
                    Wire.endTransmission();                     // End the I2C data transmission.
                    break;
                }
            }
          
            Serial.println("rtd: " + String(RTD_data));          
            publish("rtd", RTD_data);
        } 
        serial_event = false;
    }
}

// Send data to the server
void sendData(char endpoint[]){
    // Connect to the server
    connect();

    // Send data to the server
    blink(led, 1000, 1000, 1);
    client.println(endpoint);
    client.println("Host: 34.210.238.35");
    client.println("Content-Length: 0");
    client.println();
    
    // Disconnect from the server
    disconnect();
}

void publish(char key[], char value[]){
    Spark.publish(String(key), String(value));
}

void connect(){    
    if (client.connect(server, port)){
        // beacon(led, 4);
        Spark.publish("connected", "True");
    }
    else{
        // blink(led, 2500, 500, 4);
        Spark.publish("connected", "False");
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