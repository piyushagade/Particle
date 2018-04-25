#include "Particle.h"
//#include "Wire.h"                //enable I2C.
#define rtd_address 102             //default I2C ID number for EZO RTD Circuit.



char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.   
byte received_from_computer=0;   //we need to know how many characters have been received.    
byte serial_event=0;             //a flag to signal when data has been received from the pc/mac/other. 
byte code=0;                     //used to hold the I2C response code. 
char RTD_data[20];               //we make a 20 byte character array to hold incoming data from the RTD circuit. 
byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
byte i=0;                        //counter used for RTD_data array. 
int time_=600;                   //used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 
float tmp_float;                 //float var used to hold the float value of the RTD. 


void setup()                    //hardware initialization.                
{
  Serial.begin(9600);           //enable serial port.  
  Wire.begin();                 //enable I2C port.
  Serial.println("Setup completed"); 
}


void serialEvent(){                                                               //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.    
    received_from_computer=Serial.readBytesUntil(13,computerdata,20);     //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.      
    computerdata[received_from_computer]=0;                               //stop the buffer from transmitting leftovers or garbage.
    serial_event = true;                                                 //set the serial event flag.
    Spark.publish("Serial event", "true");
}    
        
void loop(){ 
    if (serial_event == true) {                                                     //if a command was sent to the RTD circuit.
        computerdata[0] = tolower(computerdata[0]);                               //we make sure the first char in the string is lower case
        if(computerdata[0]=='c'||computerdata[0]=='r')
            time_=600;                                                               //if a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.  
        else time_=300;                                                            //if any other command has been sent we wait only 300ms.
       
    
        Wire.beginTransmission(rtd_address); //call the circuit by its ID number.  
        Wire.write(computerdata);        //transmit the command that was sent through the serial port.
        Wire.endTransmission();          //end the I2C data transmission. 
        
    
        if (strcmp(computerdata, "sleep") != 0) {                               //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                                                                //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the RTD circuit.
          
            delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction. 
            
            Wire.requestFrom(rtd_address,20,1); //call the circuit and request 20 bytes (this may be more than we need)
            code=Wire.read();               //the first byte is the response code, we read this separately.  
            
            switch (code){                  //switch case based on what the response code is.  
                case 1:                       //decimal 1.  
                    Serial.println("Success");  //means the command was successful.
                    break;                        //exits the switch case.
                
                case 2:                        //decimal 2. 
                    Serial.println("Failed");    //means the command has failed.
                    break;                         //exits the switch case.
                
                case 254:                      //decimal 254.
                    Serial.println("Pending");   //means the command has not yet been finished calculating.
                    break;                         //exits the switch case.
                
                case 255:                      //decimal 255.
                    Serial.println("No Data");   //means there is no further data to send.
                    break;                         //exits the switch case.
            }
            
         
            while(Wire.available()){          //are there bytes to receive.  
                in_char = Wire.read();           //receive a byte.
                RTD_data[i]= in_char;            //load this byte into our array.
                i+=1;                            //incur the counter for the array element. 
                if(in_char==0){                 //if we see that we have been sent a null command. 
                    i=0;                        //reset the counter i to 0.
                    Wire.endTransmission();     //end the I2C data transmission.
                    break;                      //exit the while loop.
                }
            }
          
            Serial.println("rtd: " + String(RTD_data));          //print the data.
            Spark.publish("rtd", RTD_data);
        } 
        serial_event=false;               //reset the serial event flag.
    }

  //Uncomment this section if you want to take the RTD value and convert it into floating point number.
  //RTD_float=atof(RTD_data); 
}
 