#include <Arduino.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include "CRC.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CutDownNichromeTime 5000 //milli sec
#define CutDownPin 34
#define ParachutePin 35
bool nichromeON = false;
unsigned long CutDownStart = 0;

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

typedef struct recieved_data{
    int cutdown_time;
    bool update_cutdown_time;
    bool RunTimer;
    bool trigger_cutdown;
    bool trigger_parachute;   
} recieved_data;
recieved_data rx_data;




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
          oled.print('<');
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
    }

    void menu1(){
      oled.print("Packet:");
      oled.print(myPacket.packetcount);
      oled.print(":");
      //oled.print();
      oled.print(":");
      //oled.println();
      oled.print("lat:");
      oled.println(myPacket.lat ,10);
      oled.print("lng:");
      oled.println(myPacket.lng ,10);
      oled.print("Alt:");
      oled.println(myPacket.alt);
      oled.print("Bat V:");
      //oled.println();
      oled.print("Temp: ");
      //oled.println();
      oled.setCursor(0,56);
      oled.print("1");
    }

    void menu2Control(){
      if(is_selected){
        switch (cursor)
        {
        case 0:
          if(selectCount > 0){
            myPacket.cutdown_time = myPacket.cutdown_time + 10;
            selectCount = 0;
          }
          if(selectCount < 0){
            myPacket.cutdown_time = myPacket.cutdown_time - 10;
            selectCount = 0;
          }
          break;
        case 1:
          if(selectCount > 0){
            myPacket.timer_running = !myPacket.timer_running;
            selectCount = 0;
          }
          if(selectCount < 0){
            myPacket.timer_running = !myPacket.timer_running;
            selectCount = 0;
          }
        case 2:
          if(selectCount > 0){
            
            selectCount = 0;
          }
          if(selectCount < 0){
            
            selectCount = 0;
          }
          break;
        case 3:
          if(selectCount > 0){
            myPacket.parachute_status = !myPacket.parachute_status;
            selectCount = 0;
          }
          if(selectCount < 0){
            myPacket.parachute_status = !myPacket.parachute_status;
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
      oled.print("Cutdown Time:");
      oled.print(myPacket.cutdown_time);

      oled.setCursor(8,8);
      oled.print("Timer Running:");
      oled.print(myPacket.timer_running);

      oled.setCursor(8,16);
      oled.print("Cutdown Status:");
      oled.print(myPacket.cutdown_status);

      oled.setCursor(8,24);
      oled.print("Parachute Status:");
      oled.print(myPacket.parachute_status);
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
      Serial.println(selectCount);
    }

};
menu MyMenu;







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
    uint32_t crc = CRC::Calculate(&rx_data, sizeof(rx_data), CRC::CRC_32());
    uint8_t payload[sizeof(rx_data)+sizeof(crc)];
    memcpy(&payload[0], &rx_data, sizeof(rx_data));
    memcpy(&payload[sizeof(rx_data)], &crc, sizeof(crc));
    rfd_PacketSerial.send(payload, sizeof(payload));
}

void setup() {
  rfd.begin(57600, SERIAL_8N1, 16, 17);

  Serial.begin(115200);
  rfd_PacketSerial.setStream(&rfd);
  rfd_PacketSerial.setPacketHandler(&rfd_PacketReceived);

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

  
}

void loop() {
  rfd_PacketSerial.update();

  MyMenu.update();

  updateButton();

  //run at 10hz
  unsigned long currentMillis = millis();
  if(currentMillis - MillisCount1 >= 100){
    MillisCount1 = currentMillis;
    //Send_packet();
    
  }

  //run at 1hz
  if(currentMillis - MillisCount2 >= 1000){
    MillisCount2 = currentMillis;
    
  }
    
}