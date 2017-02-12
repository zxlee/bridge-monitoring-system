
//Include library here
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define LED 13

//Instantiate RF
RF24 radio(7,8);                //radio(CE, CSN)
RF24Network network(radio);     //Network uses that radio

//Store payload locally
struct local_payload {
  short slave_address;    //Used for master identifier
  short water_sensor;         //Water level measurement
  short weight;               //Load cell strain
  short xaxis;            //Accelerometer X axis
  short yaxis;            //Accelerometer Y axis
  short zaxis;            //Accelerometer Z axis
  boolean status;
};

local_payload local_data; 

//Declare constant here
const uint16_t this_node  = 00;
const uint16_t node_1_add = 01;
const uint16_t node_2_add = 02;
const uint16_t node_3_add = 03;

//Alert threshold
const short accelerometer_threshold = 1;
const short water_level_threshold = 3;
const short load_cell_threshold = 7;


//State machine state
enum { PING, RECEIVE, PROCESS, RESPOND, TOHOST, DONE} state = PING; 


//RF24 Payload Section
struct payload_ping {
  boolean ready;
};

struct payload_receive {
  short slave_address;    //Used for master identifier
  short water_sensor;         //Water level measurement
  short weight;               //Load cell strain
  short xaxis;            //Accelerometer X axis
  short yaxis;            //Accelerometer Y axis
  short zaxis;            //Accelerometer Z axiS
};

struct payload_respond {
  boolean bridge;       //0 = Off, 1 = On
};

//=====================================================
//=================Code begin here=====================

void setup(void)
{
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  //Serial.println("Start");

  SPI.begin();
  radio.begin();
  network.begin(120, this_node);   //begin(channel, node address)
}

void loop()
{
  state_machine(node_1_add);
  state_machine(node_2_add);
  state_machine(node_3_add);
}

void state_machine(uint16_t address_to_ping)
{  
  while(state != DONE)
  {
    network.update();
    switch(state)
    {
      case PING:                        //this state is used to ping the sensor node
      {
        digitalWrite(LED, HIGH);
        payload_ping payload1 = {true};
        RF24NetworkHeader header(address_to_ping);
        
        if(network.write(header, &payload1, sizeof(payload1)))
        {
          //Serial.println("Ping Success");
          state = RECEIVE;                   //if success ping only change state
        } 
        else
        {
          //Serial.println("Ping Fail");   //fail to ping, retry
        }
      }   
      break;
      
      case RECEIVE:
      {
        digitalWrite(LED, LOW);
        
        char buffer[50];
        
        while(network.available())
        {          
          RF24NetworkHeader header;
          payload_receive payload;
          network.read(header, &payload, sizeof(payload));
          
          //Store a local copy of payload, once out of this state,
          //the data on payload will be cleared
          local_data.slave_address = payload.slave_address;
          local_data.water_sensor = payload.water_sensor;
          local_data.weight = payload.weight;
          local_data.xaxis = payload.xaxis;
          local_data.yaxis = payload.yaxis;
          local_data.zaxis = payload.zaxis;

          state = PROCESS;
          
        }
      }
      break;

      case PROCESS:
      {
        danger_processing();        
        state = TOHOST;
      }
      break;
      
      case RESPOND:
      {
        payload_respond payload = {!local_data.status};

        RF24NetworkHeader header(address_to_ping);
        bool ok = network.write(header, &payload, sizeof(payload));
        if(ok)
        {
          //Serial.println("Transmit Respond Success");
          state = DONE;
        }
        else
        {
          //Serial.println("Transmit Respond Fail");
        }
      }
      break; 

      case TOHOST:
      {
        char buffer[50];
        sprintf(buffer, "$%d,%d,%d,%d,%d,%d,%d\r\n", local_data.slave_address, local_data.water_sensor, local_data.weight,
        local_data.xaxis, local_data.yaxis, local_data.zaxis, local_data.status);
        Serial.write(buffer);
        state = RESPOND;     
      }
      break; 
       
      case DONE:
      {
        //Do nothing
       // state = PING;
      }
      break;  
    }
  delay(10);
  }
  state = PING;         //before exit loop, reinitialize the state to PING when next enter
}


void danger_processing()
{
  //create variable for local function processing
  int x = local_data.xaxis;
  int y = local_data.yaxis;
  int z = local_data.zaxis;
  
  float x_axis, y_axis, z_axis;
  float load_cell = local_data.weight;
  short water_level = local_data.water_sensor;

  float movement_rms;

  //convert the reading into voltage
  x_axis = (x*0.1875)/1000;
  y_axis = (y*0.1875)/1000;
  z_axis = (z*0.1875)/1000;
//  Serial.print("voltage: ");
//  Serial.print(x_axis);
//  Serial.print(",");
//  Serial.print(y_axis);
//  Serial.print(",");
//  Serial.println(z_axis);

  //convert voltage into g reading
  x_axis = x_axis*(7.2/3.3)-3.6;
  y_axis = y_axis*(7.2/3.3)-3.6;
  z_axis = z_axis*(7.2/3.3)-3.6;
//  Serial.print("g: ");
//  Serial.print(x_axis);
//  Serial.print(",");
//  Serial.print(y_axis);
//  Serial.print(",");
//  Serial.println(z_axis);

  //use the rms value of three axis accelerometer for detection
  movement_rms = sqrt((x_axis*x_axis) + (y_axis*y_axis) + (z_axis*z_axis));
  //Serial.println(movement_rms);

  //Rescale the load cell reading
  load_cell = load_cell/100;

  if(water_level == water_level_threshold || movement_rms > accelerometer_threshold || load_cell > load_cell_threshold)
  //if(load_cell > load_cell_threshold)
  {
    local_data.status = true;
  }
  else
  {
    local_data.status = false;
  }  
}

