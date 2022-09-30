#include <Arduino.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include "CRC.h"

#define CutDownNichromeTime 5000 //milli sec
#define CutDownPin 34
#define ParachutePin 35
bool nichromeON = false;
unsigned long CutDownStart = 0;


static const uint32_t GPSBaud = 9600;
PacketSerial rfd_PacketSerial;
PacketSerial lora_PacketSerial;
HardwareSerial lora(2);
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

void lora_PacketReceived(const uint8_t* buffer, size_t size)
{
  uint32_t crc1 = CRC::Calculate(buffer, size, CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buffer[sizeof(myPacket)], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&myPacket, &buffer, sizeof(myPacket)); 
  }
  else{
    myPacket.lora_bad_packet++;
  }
}

void Send_packet(){
    uint32_t crc = CRC::Calculate(&rx_data, sizeof(rx_data), CRC::CRC_32());
    uint8_t payload[sizeof(rx_data)+sizeof(crc)];
    memcpy(&payload[0], &rx_data, sizeof(rx_data));
    memcpy(&payload[sizeof(rx_data)], &crc, sizeof(crc));
    rfd_PacketSerial.send(payload, sizeof(payload));
    lora_PacketSerial.send(payload, sizeof(payload));
}

void setup() {
  rfd.begin(57600, SERIAL_8N1, 17, 16);
  lora.begin(57600, SERIAL_8N1, 18, 19);
  Serial.begin(115200);
  rfd_PacketSerial.setStream(&rfd);
  rfd_PacketSerial.setPacketHandler(&rfd_PacketReceived);
  lora_PacketSerial.setStream(&lora);
  lora_PacketSerial.setPacketHandler(&lora_PacketReceived);

  myPacket.cutdown_status = false;
  myPacket.cutdown_time = 3600;
  myPacket.parachute_status = false;

}

void loop() {
  rfd_PacketSerial.update();
  lora_PacketSerial.update();

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