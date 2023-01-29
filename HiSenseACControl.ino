#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Kelon.h>
#include <NullSpaceLib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h> //Adafruit RTClib per DS3231 e DateTime
#include <ArduinoWebsockets.h> //Downloading ArduinoWebsockets@0.5.3
#include <ArduinoJson.h> //configuratore: https://arduinojson.org/v6/assistant/#/step1
#include <Arduino.h>
#ifdef ESP8266 
       #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   
       #include <WiFi.h>
#endif
#include "SinricPro.h"
#include "SinricProWindowAC.h"

#define REMOTE_NODE_ID 3

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif 

const char* ssid = "ssid"; //Enter SSID
const char* password = "pass"; //Enter Password
#define APP_KEY           "api"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "api"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define ACUNIT_ID         "api"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         115200                     // Change baudrate to your need

const char* websockets_server_host = "192.168.178.102"; //Enter server adress
const uint16_t websockets_server_port = 8080; // Enter server port

class DataHolder
{
  public:
    DataHolder()
    {
      //init
      this->node_temperature=25.0;
      this->node_humidity=80;
      this->ac_setpoint_temperature_hot=round((kKelonMaxTemp+kKelonMinTemp)/2.0);
      this->ac_setpoint_temperature_cold = this->ac_setpoint_temperature_hot;
      this->ac_setpoint_humidity_dry = 50; //%
      this->ac_fan_speed=0;
      this->ac_mode=kKelonModeHeat;
      this->ac_swing_vertical=true;
    }
    float node_temperature;
    float node_humidity;
    float ac_setpoint_temperature_hot;
    float ac_setpoint_temperature_cold;
    float ac_setpoint_humidity_dry;
    bool ac_power_state;
    int ac_fan_speed;
    int ac_mode;
    bool ac_swing_vertical;
};
DataHolder g_DataHolder;

#define DHTPIN 2      // Digital pin connected to the DHT sensor GPIO2=D4
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t g_dht_event;
TimerC timer_sampling_temp_hum;
uint32_t sampling_time_temp_hum;

const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRKelonAc ac(kIrLed);  // Set the GPIO to be used for sending messages.

bool first_run = true;

//WEBSOCKET CLIENT
using namespace websockets;
TimerC timer_websocket_client_send;
#define sampling_time_websocket_client_send 1000 //ms
TimerC timer_websocket_client_rcv;
#define sampling_time_websocket_client_rcv 500 //ms
WebsocketsClient client_ws;

void printState() {
  // Display the settings.
  Serial.println("Kelon A/C remote is in the following state:");
  Serial.printf("  %s\n", ac.toString().c_str());
  // Display the encoded IR sequence.
  /*
  unsigned char* ir_code = ac.getRaw();
  Serial.print("IR Code: 0x");
  for (uint8_t i = 0; i < kKelon168StateLength ; i++)
    Serial.printf("%02X", ir_code[i]);
  Serial.println();
  */
}

void ConnectWiFi()
{
  //CONNESSIONE AL WIFI
  WiFi.begin(ssid, password);
  //TENTATIVI DI CONNESSIONE
  int counter = 0;
  while (!WiFi.isConnected()) 
  {
    delay(200);
    Serial.println("WiFi Non Connesso...Aspetta");    
    if (++counter > 100)
    {
      //SE NON SI CONNETTE DOPO 100 TENTATIVI, RESETTA ESP 
      ESP.restart();
    }
     
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP
}

bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("AC %s turned %s\r\n", deviceId.c_str(), state?"on":"off");
  g_DataHolder.ac_power_state = state;
  if(state)
  {
    ac.setTogglePower(g_DataHolder.ac_power_state);
    ac.send();
  }
  else
  {
    ac.setTogglePower(g_DataHolder.ac_power_state);
    ac.ensurePower(true);
  }
    
  printState();
  return true; // request handled properly
}
bool onTargetTemperature(const String &deviceId, float &temperature) {
  Serial.printf("AC %s set temperature to %f\r\n", deviceId.c_str(), temperature);
  g_DataHolder.ac_setpoint_temperature_hot = temperature;
  ac.setTemp(static_cast<uint8_t>(g_DataHolder.ac_setpoint_temperature_hot));
  ac.send();
  printState();
  return true;
}
bool onAdjustTargetTemperature(const String & deviceId, float &temperatureDelta) {
  g_DataHolder.ac_setpoint_temperature_hot += temperatureDelta;  // calculate absolut temperature
  Serial.printf("AC %s changed temperature about %f to %f", deviceId.c_str(), temperatureDelta, g_DataHolder.ac_setpoint_temperature_hot);
  temperatureDelta = g_DataHolder.ac_setpoint_temperature_hot; // return absolut temperature
  ac.setTemp(static_cast<uint8_t>(g_DataHolder.ac_setpoint_temperature_hot));
  ac.send();
  printState();
  return true;
}
bool onThermostatMode(const String &deviceId, String &mode) {
  Serial.printf("AC %s set to mode %s\r\n", deviceId.c_str(), mode.c_str());
  String mode_ = String(mode.c_str());
  mode_.toLowerCase();
  //Serial.println("### "+mode_);
  if(mode_=="cool")
  {
      g_DataHolder.ac_mode = kKelonModeCool;
      ac.setMode(g_DataHolder.ac_mode);
      printState();
      ac.send();
  }
  else if(mode_=="heat")
  {
      g_DataHolder.ac_mode = kKelonModeHeat;
      ac.setMode(g_DataHolder.ac_mode);
      printState();
      ac.send();
  }
  else if(mode_=="eco")
  {
    //fan
    g_DataHolder.ac_mode = kKelonModeFan;
    ac.setMode(g_DataHolder.ac_mode);
    printState();
    ac.send();
  }
  else if(mode_=="auto")
  {
    //dry
    g_DataHolder.ac_mode = kKelonModeDry;
    ac.setMode(g_DataHolder.ac_mode);
    printState();
    ac.send();

  }
  return true;
}
bool onRangeValue(const String &deviceId, int &rangeValue) {
  Serial.printf("Fan speed set to %d\r\n", rangeValue);
  g_DataHolder.ac_fan_speed = rangeValue;
  ac.setFan(g_DataHolder.ac_fan_speed);
  ac.send();
  printState();
  return true;
}
bool onAdjustRangeValue(const String &deviceId, int &valueDelta) {
  g_DataHolder.ac_fan_speed += valueDelta;
  Serial.printf("Fan speed changed about %d to %d\r\n", valueDelta, g_DataHolder.ac_fan_speed);
  valueDelta = g_DataHolder.ac_fan_speed;
  ac.setFan(g_DataHolder.ac_fan_speed);
  ac.send();
  printState();
  return true;
}
void setupSinricPro() {
  SinricProWindowAC &myAcUnit = SinricPro[ACUNIT_ID];
  myAcUnit.onPowerState(onPowerState);
  myAcUnit.onTargetTemperature(onTargetTemperature);
  myAcUnit.onAdjustTargetTemperature(onAdjustTargetTemperature);
  myAcUnit.onThermostatMode(onThermostatMode);
  myAcUnit.onRangeValue(onRangeValue);
  myAcUnit.onAdjustRangeValue(onAdjustRangeValue);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
}
int StringSplit(String (&strs)[20], String& str, char sep=',')
{
  //String strs[20];//hardcoded max size of the split
  int StringCount = 0;
   // Split the string into substrings
  while (str.length() > 0)
  {
    int index = str.indexOf(sep);
    if (index == -1) // No space found
    {
      strs[StringCount++] = str;
      break;
    }
    else
    {
      strs[StringCount++] = str.substring(0, index);
      str = str.substring(index+1);
    }
  }
  /*
    // Show the resulting substrings
    for (int i = 0; i < StringCount; i++)
    {
      Serial.print(i);
      Serial.print(": \"");
      Serial.print(strs[i]);
      Serial.println("\"");
    }
  */
  return StringCount;
}
//WEBSOCKET
void connectWSClientToServer()
{
    // try to connect to Websockets server
    bool connected = client_ws.connect(websockets_server_host, websockets_server_port, "/");
    if(connected) {
        Serial.println("Connecetd!");
        //client_ws.send("Hello Server");
    } else {
        Serial.println("Not Connected!");
    }
    
    // run callback when messages are received
    client_ws.onMessage([&](WebsocketsMessage message) {
        Serial.print("Got Message: ");
        Serial.println(message.data());

        //unpack command from Central Station
        String msg = String(message.data());
        if(msg[0]=='!')
        {
          msg = msg.substring(1);//remove "!"
          String strs[20];//hardcoded max size of the split
          int num_split = StringSplit(strs,msg,',');
          if(strs[0].toInt()==REMOTE_NODE_ID)
          {
            //mode
            String strmode = strs[1];
            int mode = 0;
            float setpoint;
            if (strmode == "Heat")
            {
              mode = kKelonModeHeat;
              setpoint = strs[2].toFloat();
            }
            else if(strmode == "Cool")
            {
              mode = kKelonModeCool;
              setpoint = strs[3].toFloat();              
            }
            else if(strmode == "Dry")
            {
              mode = kKelonModeDry;
              setpoint = strs[2].toFloat();
            }
            else
            {
              mode = kKelonModeDry;
              setpoint = strs[2].toFloat();
            }
            
            
            bool cmd_OnOff = (bool)strs[4].toInt();
            int fanspeed = strs[5].toInt();

            //check with current state
            bool send_to_ac=false;
            if(g_DataHolder.ac_fan_speed!=fanspeed)
            {
              g_DataHolder.ac_fan_speed=fanspeed;
              send_to_ac=true;
            }
            if(g_DataHolder.ac_mode!=mode)
            {
              g_DataHolder.ac_mode=mode;
              send_to_ac=true;
            }
            if(g_DataHolder.ac_mode==kKelonModeHeat)
            {
              if( static_cast<int>(g_DataHolder.ac_setpoint_temperature_hot*10.0) != static_cast<int>(setpoint*10.0) )
              {
                g_DataHolder.ac_setpoint_temperature_hot=setpoint;
                send_to_ac=true;                
              }
            }
            else if(g_DataHolder.ac_mode==kKelonModeCool)
            {
              if( static_cast<int>(g_DataHolder.ac_setpoint_temperature_cold*10.0) != static_cast<int>(setpoint*10.0) )
              {
                g_DataHolder.ac_setpoint_temperature_cold=setpoint;
                send_to_ac=true;
              }
            }
            if(g_DataHolder.ac_power_state!=cmd_OnOff)
            {
              g_DataHolder.ac_power_state=cmd_OnOff;
              if(g_DataHolder.ac_power_state)
              {
                ac.setTogglePower(g_DataHolder.ac_power_state);
                ac.send();
              }
              else
              {
                ac.setTogglePower(g_DataHolder.ac_power_state);
                ac.ensurePower(true);
              }
              send_to_ac=true;
            }

            if(send_to_ac)
            {
              //send to ac the new state
              ac.setTemp(static_cast<uint8_t>(setpoint));
              ac.setMode(g_DataHolder.ac_mode);
              ac.setFan(g_DataHolder.ac_fan_speed);
              ac.send();
              printState();
            }
            

          }

        }


    });
}

void clientWSSendDataToServer()
{
  if(timer_websocket_client_send.getET() >= sampling_time_websocket_client_send)
  {
    timer_websocket_client_send.reset();
    //send data over websocket to server
    //TODO: BETTER TO USE JSON
    //I USE MY COSTUM STRING PROTOCOL

    String fanspeed;
    if(g_DataHolder.ac_fan_speed==0)
    {
      fanspeed="Auto";
    }
    else
    {
      fanspeed = String(g_DataHolder.ac_fan_speed);
    }
    String acmode;
    switch (g_DataHolder.ac_mode) 
    {
      case kKelonModeCool:
      {
        acmode="Cool";
        break;
      }
      case kKelonModeHeat:
      {
        acmode="Heat";
        break;
      }
      case kKelonModeDry:
      {
        acmode="Dry";
        break;
      }
      default:
      {
        acmode="Heat";
        break;
      }

    }

    char msg[80];
    sprintf(msg,"!%d,%s,%d,%s,%s,%s,%s,%s,%s,%s", REMOTE_NODE_ID,String(g_DataHolder.node_temperature,1),static_cast<int>(g_DataHolder.node_humidity),
    g_DataHolder.ac_power_state ? "On":"Off", fanspeed, acmode, 
    String(g_DataHolder.ac_setpoint_temperature_hot,1),String(g_DataHolder.ac_setpoint_temperature_cold,1),
    String(g_DataHolder.ac_setpoint_humidity_dry,0),g_DataHolder.ac_swing_vertical ? "On":"Off");
    client_ws.send(msg);
    
  }
}

bool read_temp_hum_node_station(sensors_event_t& dht_event)
{
  if(timer_sampling_temp_hum.getET()>=sampling_time_temp_hum)
  {
    //digitalWrite(PIN_RELE,!digitalRead(PIN_RELE));
    timer_sampling_temp_hum.reset();
    dht.temperature().getEvent(&dht_event);
    if (isnan(dht_event.temperature)) {
      //Serial.println(F("Error reading temperature!"));
      return false;
    }
    else {
      g_DataHolder.node_temperature = dht_event.temperature;

      //Serial.print(F("Temperature: "));
      //Serial.print(g_dht_event.temperature);
      //Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&dht_event);
    if (isnan(dht_event.relative_humidity)) {
      //Serial.println(F("Error reading humidity!"));
      return false;
    }
    else {
      g_DataHolder.node_humidity = dht_event.relative_humidity;
      //Serial.print(F("Humidity: "));
      //Serial.print(g_dht_event.relative_humidity);
      //Serial.println(F("%"));      
    }
  }
  return true;
}
void clientCheckForIncomingMsgs()
{
  if(timer_websocket_client_rcv.getET() >= sampling_time_websocket_client_rcv)
  {
    timer_websocket_client_rcv.reset();
    // let the websockets client_ws check for incoming messages
    if(client_ws.available()) {
        client_ws.poll();
    }
  }
}

void setup() {
  ac.begin();
  Serial.begin(BAUD_RATE);
  delay(200);
  ConnectWiFi();
  delay(100);
  setupSinricPro();
  Serial.println("Default state of the remote.");
  printState();
  Serial.println("Setting desired state for A/C.");
  //ac.on(); //NOT USED
  //ac.ensurePower(true);
  ac.setTogglePower(g_DataHolder.ac_power_state);
  ac.setFan(g_DataHolder.ac_fan_speed); //0=Auto //1-3 SPEED
  /* MODES
  const uint8_t kKelonModeHeat = 0;
  const uint8_t kKelonModeSmart = 1;  // (temp = 26C, but not shown)
  const uint8_t kKelonModeCool = 2;
  const uint8_t kKelonModeDry = 3;    // (temp = 25C, but not shown)
  const uint8_t kKelonModeFan = 4;    // (temp = 25C, but not shown)
  */
  //ac.setMode(kKelonModeHeat);
  ac.setMode(g_DataHolder.ac_mode);
  ac.setTemp(g_DataHolder.ac_setpoint_temperature_hot);
  ac.setToggleSwingVertical(g_DataHolder.ac_swing_vertical);
  //ac.setSwingHorizontal(true);
  //ac.setXFan(true);
  //ac.setIonFilter(false);
  //ac.setLight(true);

  // Initialize device.
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  sampling_time_temp_hum = sensor.min_delay / 1000; //ms

  //websocket
  connectWSClientToServer();

  timer_sampling_temp_hum.start();
  timer_websocket_client_send.start();
  timer_websocket_client_rcv.start();
}

void loop() 
{
  SinricPro.handle();
  bool ok = read_temp_hum_node_station(g_dht_event);
  if(ok)
  {
    clientWSSendDataToServer();
  }

  clientCheckForIncomingMsgs();
}
