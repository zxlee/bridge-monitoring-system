
//Include library here
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <hx711.h>
#include <Servo.h>

//Instantiate RF24 radio
RF24 radio(7,8);
RF24Network network(radio);

//Instantiate HX711 load cell
Hx711 scale(A1, A0);
int gram;

//Instantiate ADS1115 External ADC
Adafruit_ADS1115 ads; 

//Instantiate Servo
Servo gate; 
const int open_position = 0;
const int close_position = 90;

//Declare constant here
const uint16_t who_am_i = 02;   //Node address
const uint16_t master   = 00;   //Hub address

//ADC variables
int16_t adc0,adc1,adc2,adc3;
float 	ADC0,ADC1,ADC2,ADC3;


//Water Sensor
const int water_level_sensor = A6; //Water level sensor connected to pin A6
int water_state = 0;

//Servo motor
int servo_pos = 0;          //Servo motor position


//State machine state
enum {INIT, MEAS_WATER_LEVEL, MEAS_MOVEMENT, MEAS_WEIGHT, PROCESS, PONG, TRANSMIT, RESPOND} global_state = INIT;   
//initial state is initialization


//RF24 Payload Section
struct payload_ping {
  boolean ready;
};

struct payload_transmit {
  uint16_t slave_address;		//Used for master identifier
  int water_sensor;				  //Water level measurement
  int weight;					      //Load cell strain
  int16_t xaxis;				    //Accelerometer X axis
  int16_t yaxis;				    //Accelerometer Y axis
  int16_t zaxis;				    //Accelerometer Z axis
  
};

struct payload_respond {
  boolean bridge_state;       //0 = Off, 1 = On
};

//=====================================================
//=================Code begin here=====================

void setup()
{
  Serial.begin(115200);
  Serial.print("Start Node ");
  Serial.println(who_am_i);
  

  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();
  
  gate.attach(9);

  gate.write(0);

  SPI.begin();
  radio.begin();
  network.begin(120, who_am_i);      //begin(channel, node address); 
  
}

void loop()
{
  network.update();
 // Serial.print("State: ");
 // Serial.println(global_state);
  switch(global_state)
  {
    case INIT:                        //code that only run once
    {  
      global_state = MEAS_WATER_LEVEL;//Once the code finish execute, define the next state
    }
    break;

    case MEAS_WATER_LEVEL:    				//Acquire water level sensor measurement
    {
      Measure_Waterlevel();
      global_state = MEAS_MOVEMENT;
    }
    break;

    case MEAS_MOVEMENT:       //Acquire three axis measurement from accelerometer
    {
      Measure_Movement();
      global_state = MEAS_WEIGHT;
    }
    break;

    case MEAS_WEIGHT:         //Acquire measurement from load cell
    {
      gram = scale.getGram()*100;      
      global_state = PROCESS;
    }
    break;

    case PROCESS:
    {
      //Uncomment to display readings for debug
	    //ADC
      //Serial.print("AIN0: "); Serial.println(adc0);
      //Serial.print("AIN1: "); Serial.println(adc1);
      //Serial.print("AIN2: "); Serial.println(adc2);
      //Serial.println(" ");

      // Serial.print("Water level: ");Serial.println(danger);

      //Serial.print("Weight: ");Serial.println(gram);
      //water_sensor = danger;
      //weight= gram;
      //xaxis= adc0;
      //yaxis= adc1;
      //zaxis= adc2;
      
   // Serial.println("Process");
	    global_state = PONG;
    }
    break;

    case PONG:
    {
      //Wait for the Master to ping on this node, if this node is not
      //ready in this state, the master will continue to ping
      //Serial.println("PONG");

       
      
      while(network.available())
     {
        RF24NetworkHeader header;
        payload_ping payload;
        network.read(header, &payload, sizeof(payload));
        Serial.println("Acknowledged");
        //Serial.println(payload.ready);  
        global_state = TRANSMIT;
      }
    }
    break;

    case TRANSMIT:
    { 
      //payload format <node address, water_state, strain, x_axis, y_axis, z_axis>      
      payload_transmit payload = {who_am_i,water_state,gram,adc0,adc1,adc2};
      
      RF24NetworkHeader  header(master);
      bool ok = network.write(header, &payload, sizeof(payload));
      //Serial.println(sizeof(payload));
      
      if(ok)
      {
       // Serial.println("Transmit success");
        global_state = RESPOND;
      }
      else
      {
        Serial.println("Transmit fail");
      }    
    }
    break;


    case RESPOND:
    {
       Serial.println("Respond");
               
       while(network.available())
       {          
         Serial.println("Receieving");
         RF24NetworkHeader header;
         payload_respond payload;
         network.read(header, &payload, sizeof(payload));
         Serial.print("Bridge: ");
         Serial.println(payload.bridge_state);
         if(payload.bridge_state == true)
         {
            close_gate();      
         }
         else if(payload.bridge_state == false)
         {
            open_gate();       
         }
         global_state = MEAS_WATER_LEVEL;
       }
       
    }
    break;

    default:
    Serial.println("Unknown State");
    break;
    
  }

  delay(10);    //10 milli seconds delay between each state transition
}


void Measure_Movement()
{
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
}


void Measure_Waterlevel()
{
  int value = analogRead(water_level_sensor);
  
  if (value<=10)
  { 
    water_state = 1;
    //Serial.println("sea level low.Bridge ok"); // water not touching sensor
  }
  else if (value>10 && value<=300)
  { 
    water_state = 2;
    //Serial.println("Caution!");                //water touches below 10mm
  }
  else if (value>300)
  { 
    water_state = 3;
    //Serial.println("Bridge close");            //water touches more than 10mm
  } 
}

void close_gate()
{
  gate.write(close_position);
//  for(int x = open_position; x <= close_position; x=x+10)
//  {
//    gate.write(x);
//    delay(10);      
//  }
}

void open_gate()
{
  gate.write(open_position);
//  for(int x = close_position; x >= open_position; x=x-10)
//  {
//    gate.write(x);
//    delay(10);
//  }
}





