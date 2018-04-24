int led = D7;
TCPClient client;
byte server[] = { 34, 210, 238, 35 }; // AWS
int port = 1221;
bool active = true;

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

// Device setup
void setup(){
    pinMode(led, OUTPUT);
}

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
    if(active){
        Cellular.off();
    }
    else{
        Cellular.on();
    }
    active = !active;
}

// Collect sensor data
void getData(){

}

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