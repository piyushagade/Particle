int led = D7;
TCPClient client;
byte server[] = { 34, 210, 238, 35 }; // AWS
int port = 1221;
bool active = true;


void blink(int led, int on_time, int off_time, int repeatations){
    for(int i = 0; i < repeatations; i++){
        digitalWrite(led, HIGH);
        delay(on_time);
        digitalWrite(led, LOW);
        delay(off_time);
    }
}

void beacon(int led, int repeatations){
    for(int i = 0; i < repeatations; i++){
        digitalWrite(led, HIGH);
        delay(50);
        digitalWrite(led, LOW);
        delay(1950);
    }
}

void setup(){
    pinMode(led, OUTPUT);
    
    if (client.connect(server, port)){
        beacon(led, 4);
        Spark.publish("connected", "True");
    }
    else{
        blink(led, 2500, 500, 4);
        Spark.publish("connected", "False");
    }
}

void loop(){
    // if (client.available()){
    //     char c = client.read();
    //     Serial.print(c);
    // }
    
    // if (!client.connected()){
    //     Serial.println();
    //     Serial.println("Disconnected.");
    //     client.stop();
    //     for(;;);
    // }
    sendData();
    wait(10000);
    // togglePower();
}

void togglePower(){
    if(active){
        Cellular.off();
    }
    else{
        Cellular.on();
    }
    active = !active;
}

void wait(int time){
    delay(time);
    // beacon(led, time/1000);
}

void sendData(){
    client.println(strcat(strcat("GET ", "/insert/rtd/81.01"), " HTTP/1.0"));
    client.println(strcat("Host: ", "34.210.238.35"));
    client.println("Content-Length: 0");
    client.println();
}
