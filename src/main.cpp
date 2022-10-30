#include <Arduino.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include "CRC.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <common/mavlink.h>


#define CutDownNichromeTime 5000 //milli sec
#define CutDownPin 34
#define ParachutePin 35
bool nichromeON = false;
unsigned long CutDownStart = 0;

//antenna tracker MAC
//40:91:51:BF:E1:04
//78:21:84:7C:4F:EC
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x7C, 0x4F, 0xEC};

static const uint8_t image_data_VAST_WLOGO_BLACK[384] = {
    0xfc, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0xfe, 0x00, 0x00, 0x03, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0x07, 0xfc, 0x1f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0x0f, 0xf8, 0x3f, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0x1f, 0xf0, 0x7f, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0x3f, 0xe0, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0x7f, 0xc1, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xfc, 
    0xfe, 0x00, 0x00, 0xff, 0x83, 0xff, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x01, 0xff, 0x07, 0xff, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x03, 0xfe, 0x0f, 0xff, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x07, 0xfc, 0x1f, 0xf7, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x0f, 0xf8, 0x3f, 0xe7, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x1f, 0xf0, 0x7f, 0xc7, 0xf1, 0xfc, 0x00, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x3f, 0xe0, 0xff, 0x87, 0xf1, 0xff, 0xfc, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0x7f, 0xc1, 0xff, 0x07, 0xf1, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x00, 0xff, 0x83, 0xfe, 0x07, 0xf1, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x01, 0xff, 0x07, 0xfc, 0x07, 0xf0, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x03, 0xfe, 0x0f, 0xf8, 0x07, 0xf0, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x07, 0xfc, 0x1f, 0xf0, 0x07, 0xf0, 0x7f, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x0f, 0xf8, 0x3f, 0xe0, 0x07, 0xf0, 0x00, 0xfe, 0x0f, 0xe0, 0x00, 
    0xfe, 0x1f, 0xf0, 0x7f, 0xc0, 0x07, 0xf0, 0x00, 0x7e, 0x0f, 0xe0, 0x00, 
    0xfe, 0x3f, 0xe0, 0xff, 0x80, 0x07, 0xf0, 0x00, 0x7e, 0x0f, 0xe0, 0x00, 
    0xfe, 0x7f, 0xc1, 0xff, 0x00, 0x07, 0xf0, 0x00, 0x7e, 0x0f, 0xe0, 0x00, 
    0xfe, 0xff, 0x83, 0xfe, 0x00, 0x07, 0xf0, 0x00, 0x7e, 0x0f, 0xe0, 0x00, 
    0xff, 0xff, 0x07, 0xfc, 0x00, 0x07, 0xf0, 0x00, 0x7e, 0x0f, 0xe0, 0x00, 
    0xff, 0xfe, 0x0f, 0xf8, 0x00, 0x07, 0xf0, 0x00, 0xfe, 0x0f, 0xe0, 0x00, 
    0xff, 0xfc, 0x1f, 0xf0, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xff, 0xf8, 0x3f, 0xe0, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xff, 0xf0, 0x7f, 0xc0, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0xff, 0xe0, 0xff, 0x80, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0x7f, 0x81, 0xff, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xe0, 0x00, 
    0x7f, 0x03, 0xfe, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfc, 0x0f, 0xe0, 0x00
};
esp_now_peer_info_t peerInfo;


//oled setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static const uint32_t GPSBaud = 9600;
PacketSerial rfd_PacketSerial;
HardwareSerial rfd(1);

unsigned long MillisCount1 = 0;
unsigned long MillisCount2 = 0;

typedef struct packet {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
} packet;
packet myPacket;

typedef struct tx_data{
    int cutdown_time;
    bool update_cutdown_time;
    bool RunTimer;
    bool trigger_cutdown;
    bool trigger_parachute;   
} tx_data;
tx_data myTx;

typedef struct espdata {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
  int heading;
  int angle;
} espdata;
espdata espRX;

typedef struct ControllerData{
  float x;
  float y;
  char mode;
  int heading_offset;
  int angle_offset;

  int cutdown_time;
  bool update_cutdown_time;
  bool RunTimer;
  bool trigger_cutdown;
  bool trigger_parachute;
} ControllerData;
ControllerData espTX;


class button{
  private:
    int gpio;
    bool state;
    int count;
    bool invert = false;
  public:
    button(int _gpio){
      gpio = _gpio;
    }
    button(int _gpio, bool _invert){
      gpio = _gpio;
      invert = _invert;
    }
    void update(){
      if(invert){
        if(digitalRead(gpio) == 0 && state == false){
          state = true;
          count++;
        }
        if(digitalRead(gpio) == 1 && state == 1){
          state = false;
        }
      }
      else{
        if(digitalRead(gpio) == 1 && state == false){
          state = true;
          count++;
        }
        if(digitalRead(gpio) == 0 && state == 1){
          state = false;
        }
      }
    }
    bool isPressed(){
      return state;
    }
    int getCount(){
      return count;
    }
    void CountReset(){
      count = 0;
    }
    
    
};

button btn1(36);
button btn2(39);
button btn3(34);
button btn4(35);
button btn5(25, true);

void updateButton(){ 
  btn1.update();
  btn2.update();
  btn3.update();
  btn4.update();
  btn5.update();
}

class menu{
  private:
    int cursor = 0;
    bool is_selected = false;
    int page = 0;
    int selectCount = 0;
  public:

    
    void DisplayCursor(){
        oled.setCursor(0,cursor*8);
        if(is_selected){
          oled.print('*');
        }
        else{
          oled.print('>');
        }
    }

    void updateCursor(){
      if(btn5.getCount() > 0){
        btn5.CountReset();
        is_selected = !is_selected;
        updateDisplay();
      }
      if(!is_selected){
        selectCount = 0;
        if(btn2.getCount() > 0){
          btn2.CountReset();
          cursor++;
          if(cursor > 7){
            cursor = 7;
          }
          updateDisplay();
        }
        if(btn4.getCount() > 0){
          btn4.CountReset();
          cursor--;
          if(cursor < 0){
            cursor = 0;
          }
          updateDisplay();
        }
      }
      
    }

    void updateSelect(){
      if(is_selected){
        if(btn2.getCount() > 0){
          btn2.CountReset();
          selectCount--;
          updateDisplay();
        }
        if(btn4.getCount() > 0){
          btn4.CountReset();
          selectCount++;
          updateDisplay();
        }
      }
      
    }
    
    void menu0(){
      oled.drawBitmap(17, 16, image_data_VAST_WLOGO_BLACK, 94, 32, WHITE);
      oled.setCursor(18,56);
      oled.print("Ryan Stephenson");
    }

    void menu1(){
      oled.print("Packet:");
      oled.print(myPacket.packetcount);
      oled.print(":");
      oled.println(myPacket.rfd_bad_packet);
      oled.print("lat:");
      oled.println(myPacket.lat ,10);
      oled.print("lng:");
      oled.println(myPacket.lng ,10);
      oled.print("Alt:");
      oled.println(myPacket.alt);
      oled.print("Speed:");
      oled.println(myPacket.speed);
      oled.print("Sats:");
      oled.println(myPacket.sats);
      oled.setCursor(120,56);
      oled.print("1");
    }

    void menu2Control(){
      if(is_selected){
        switch (cursor)
        {
        case 5:
          if(selectCount > 0){
            myTx.trigger_cutdown = !myTx.trigger_cutdown;
            selectCount = 0;
          }
          if(selectCount < 0){
            myTx.trigger_cutdown = !myTx.trigger_cutdown;
            selectCount = 0;
          }
          break;
        case 6:
          if(selectCount > 0){
            myTx.trigger_parachute = !myTx.trigger_parachute;
            selectCount = 0;
          }
          if(selectCount < 0){
            myTx.trigger_parachute = !myTx.trigger_parachute;
            selectCount = 0;
          }
        case 0:
          if(selectCount > 0){
            myTx.cutdown_time = myTx.cutdown_time + 10;
            selectCount = 0;
          }
          if(selectCount < 0){
            myTx.cutdown_time = myTx.cutdown_time - 10;
            selectCount = 0;
          }
          break;
        case 1:
          if(selectCount > 0){
            myTx.update_cutdown_time = !myTx.update_cutdown_time;
            selectCount = 0;
          }
          if(selectCount < 0){
            myTx.update_cutdown_time = !myTx.update_cutdown_time;
            selectCount = 0;
          }
          break;
        case 2:
          if(selectCount > 0){
            myTx.RunTimer = !myTx.RunTimer;
            selectCount = 0;
          }
          if(selectCount < 0){
            myTx.RunTimer = !myTx.RunTimer;
            selectCount = 0;
          }
          break;
        default:
          break;
        }
      }
    }

    void menu2(){
      menu2Control();

      oled.setCursor(8,0);
      oled.print("Set CD Time:");
      oled.print(myTx.cutdown_time);

      oled.setCursor(8,8);
      oled.print("Update Timer:");
      oled.print(myTx.update_cutdown_time);

      oled.setCursor(8,16);
      if(myTx.RunTimer == true){
        oled.print("Timer Running!");
      }
      else{
        oled.print("Timmer Stoped");
      }

      oled.setCursor(8,24);
      oled.print("CD Status: ");
      if(myPacket.cutdown_status == true){
        oled.print("Recieved!");
      }
      else{
        oled.print("Waiting");
      }
      
      oled.setCursor(8,32);
      oled.print("P Status: ");
      if(myPacket.parachute_status == true){
        oled.print("Recieved!");
      }
      else{
        oled.print("Waiting");
      }


      oled.setCursor(8,40);
      oled.print("Send CD:");
      oled.print(myTx.trigger_cutdown);

      oled.setCursor(8,48);
      oled.print("Send P:");
      oled.print(myTx.trigger_parachute);

      oled.setCursor(120,56);
      oled.print("2");

      DisplayCursor();
    }

    void menu3Control(){
      if(is_selected){
        switch (cursor)
        {
        case 0:
          if(selectCount > 0){
            if(espTX.mode == 'm'){
              espTX.mode = 'a';
            }
            else{
              espTX.mode = 'm';
            }
            selectCount = 0;
          }
          if(selectCount < 0){
             if(espTX.mode == 'm'){
              espTX.mode = 'a';
            }
            else{
              espTX.mode = 'm';
            }
            selectCount = 0;
          }
          break;
        case 2:
          if(selectCount > 0){
            espTX.heading_offset = espTX.heading_offset + 1;
            selectCount = 0;
          }
          if(selectCount < 0){
            espTX.heading_offset = espTX.heading_offset - 1;
            selectCount = 0;
          }
          break;
        case 4:
          if(selectCount > 0){
            espTX.angle_offset = espTX.angle_offset + 1;
            selectCount = 0;
          }
          if(selectCount < 0){
            espTX.angle_offset = espTX.angle_offset + 1;
            selectCount = 0;
          }
          break;
        default:
          break;
        }
      }
    }
    
    void menu3(){
      menu3Control();

      oled.setCursor(8,0);
      oled.print("Mode: ");
      if(espTX.mode == 'm'){
        oled.print("manual");
      }
      if(espTX.mode == 'a'){
        oled.print("auto");
      }

      oled.setCursor(8,8);
      oled.print("Heading: ");
      oled.print(espRX.heading);

      oled.setCursor(8,16);
      oled.print("Offset: ");
      oled.print(espTX.heading_offset);

      oled.setCursor(8, 24);
      oled.print("Elevation: ");
      oled.print(espRX.angle);

      oled.setCursor(8, 32);
      oled.print("Offset: ");
      oled.print(espTX.angle_offset);

      oled.setCursor(120,56);
      oled.print("3");

      DisplayCursor();
    }
    
    void updateDisplay(){
      oled.clearDisplay();
      oled.setCursor(0,0);

      switch (page)
      {
      case 0:
        menu0();
        break;
      case 1:
        menu1();
        break;
      case 2:
        menu2();
        break;
      case 3:
        menu3();
        break;
      default:

        break;
      }
      

      oled.display();
    }

    void PageControl(){
      if(btn1.getCount() > 0){
        btn1.CountReset();
        page++;
        if(page > 3){
          page = 3;
        }
        selectCount = 0;
        is_selected = false;
        updateDisplay();
      }
      if(btn3.getCount() > 0){
        btn3.CountReset();
        page--;
        if(page < 0){
          page = 0;
        }
        selectCount = 0;
        is_selected = false;
        updateDisplay();
      }
    }

    void update(){
      updateCursor();
      PageControl();
      updateSelect();
      
    }

};
menu MyMenu;

void espNowSend(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &espTX, sizeof(espTX));
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    //Serial.println("Error sending the data");
    }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&espRX, incomingData, sizeof(espRX));
  //store data into myPacket data structure
  myPacket.alt = espRX.alt;
  myPacket.cutdown_status = espRX.cutdown_status;
  myPacket.cutdown_time = espRX.cutdown_time;
  myPacket.lat = espRX.lat;
  myPacket.lng = espRX.lng;
  myPacket.lora_bad_packet = espRX.lora_bad_packet;
  myPacket.packetcount = espRX.packetcount;
  myPacket.parachute_status = espRX.parachute_status;
  myPacket.rfd_bad_packet = espRX.rfd_bad_packet;
  myPacket.sats = espRX.sats;
  myPacket.speed = espRX.speed;
  myPacket.timer_running = espRX.timer_running;
  MyMenu.updateDisplay();
}
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}

void espNowSetup(){
  // Set device as a Wi-Fi Station
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.persistent(false);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("dummy_ssid","dummy_ssid",6,NULL,false);
  WiFi.disconnect();  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("");
    return;
  }
}

void rfd_PacketReceived(const uint8_t* buffer, size_t size)
{
  uint32_t crc1 = CRC::Calculate(&buffer[0], sizeof(myPacket), CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buffer[sizeof(myPacket)], sizeof(crc1));
  
  if(crc1 == crc2){
    memcpy(&myPacket, buffer, sizeof(myPacket));
      Serial.println(myPacket.packetcount);
  }
  else{
    myPacket.rfd_bad_packet++;
    Serial.println("bad packet");
  }
  
}

void Send_packet(){
    uint32_t crc = CRC::Calculate(&myTx, sizeof(myTx), CRC::CRC_32());
    uint8_t payload[sizeof(myTx)+sizeof(crc)];
    memcpy(&payload[0], &myTx, sizeof(myTx));
    memcpy(&payload[sizeof(myTx)], &crc, sizeof(crc));
    rfd_PacketSerial.send(payload, sizeof(payload));
}

void joystickControl(){
  if(espTX.mode == 'm'){
      //read joystick
      int x = analogRead(33);
      int y = analogRead(32);
      espTX.x = map(x, 0, 4096, -1000, 1000); 
      espTX.y = map(y, 0, 4096, -1000, 1000);
      // Send message via ESP-NOW 
      espNowSend();
  }
}

void setup() {

  espNowSetup();

  //rfd.begin(57600, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  //rfd_PacketSerial.setStream(&rfd);
  //rfd_PacketSerial.setPacketHandler(&rfd_PacketReceived);

  myPacket.cutdown_status = false;
  myPacket.cutdown_time = 3600;
  myPacket.parachute_status = false;


  pinMode(GPIO_NUM_36, INPUT);
  pinMode(GPIO_NUM_39, INPUT);
  pinMode(GPIO_NUM_34, INPUT);
  pinMode(GPIO_NUM_35, INPUT);

  pinMode(GPIO_NUM_32, INPUT);
  pinMode(GPIO_NUM_33, INPUT);
  pinMode(GPIO_NUM_25, INPUT_PULLUP);


  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.drawBitmap(17, 16, image_data_VAST_WLOGO_BLACK, 94, 32, WHITE);
  oled.display();

  espTX.mode = 'm';
  espTX.cutdown_time = 3600;
  espTX.heading_offset = 12;
  espTX.trigger_cutdown = false;
  espTX.update_cutdown_time = false;
  espTX.RunTimer = false;
}

//mavlink packet creation
void command_parameters(int8_t system_id, uint8_t component_id) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_param_value_pack(system_id, component_id, &msg, "RC_SPEED", 50, 1, 1, 0);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  Serial.write(buf, len);
}

void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {

  int16_t velx = 0; //x speed
  int16_t vely = 0; //y speed
  int16_t velz = 0; //z speed


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat * 10000000.0, lon * 10000000.0, gps_alt * 1000.0, alt * 1000.0, velx, vely, velz, heading);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat * 10000000.0, lon * 10000000.0, alt * 1000.0, gps_hdop * 100.0, 65535, groundspeed, 65535, gps_sats, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send globalgps command
  command_globalgps(system_id, component_id, upTime, lat, lon, alt, gps_alt, heading);

  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

}

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  Serial.write(buf, len);
}

void Send_Mavlink(){
  uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
  uint8_t component_id = 0;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
  uint8_t system_type = MAV_TYPE_FREE_BALLOON;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
  uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
  uint8_t system_mode = 64;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
  uint32_t custom_mode = 0;     // Usually set to 0
  uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
  uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter


  int16_t heading = 0;      // Geographical heading angle in degrees
  float lat = espRX.lat;   // GPS latitude in degrees (example: 47.123456)
  float lon = espRX.lng;   // GPS longitude in degrees
  float alt = espRX.alt;        // Relative flight altitude in m
  float groundspeed = espRX.speed; // Groundspeed in m/s
  float airspeed = 0.0;    // Airspeed in m/s
  float climbrate = 0.0;    // Climb rate in m/s, currently not working


  // GPS parameters
  int16_t gps_sats = espRX.sats;    // Number of visible GPS satellites
  int32_t gps_alt = 0.0;  // GPS altitude (Altitude above MSL)
  float gps_hdop = 100.0;     // GPS HDOP
  uint8_t fixType = 3;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix

  command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);
  command_parameters(system_id, component_id);
  command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);
}

void loop() {
  //rfd_PacketSerial.update();

  MyMenu.update();

  updateButton();

  //run at 10hz
  unsigned long currentMillis = millis();
  if(currentMillis - MillisCount1 >= 100){
    MillisCount1 = currentMillis;
    joystickControl();
    
  }

  //run at 1hz
  if(currentMillis - MillisCount2 >= 1000){
    MillisCount2 = currentMillis;
    espNowSend();
    Send_Mavlink();
  }
    
}