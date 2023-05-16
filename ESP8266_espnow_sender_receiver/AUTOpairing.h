/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/?s=esp-now
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based on JC Servaye example: https://https://github.com/Servayejc/esp8266_espnow


  ESP.rtcUserMemoryWrite(offset, &data, sizeof(data)) and 
  ESP.rtcUserMemoryRead(offset, &data, sizeof(data)) allow data to be stored in and retrieved from the RTC user memory of the chip respectively. 
  offset is measured in blocks of 4 bytes and can range from 0 to 127 blocks (total size of RTC memory is 512 bytes). 
  data should be 4-byte aligned. The stored data can be retained between deep sleep cycles, but might be lost after power cycling the chip. 
  Data stored in the first 32 blocks will be lost after performing an OTA update, because they are used by the Core internals.
*/

#ifndef AUTOpairing_H
#define AUTOpairing_H

//#define USAR_EEPROM  // no usamos la EEPROM, mejor la memoria RTC

#include <ESP8266WiFi.h>
#include <espnow.h> 
#include <EEPROM.h>
#include <string>

#include "AUTOpairing_common.h"

uint8_t broadcastAddressX[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


#define MAGIC_BYTE 0xA5
#define NOT_VALID  0xFF
#define MAGIC_CODE 4126537205
#define INVALID_CODE 0


#define BOARD_ID 2  // tiene que ser != 0, que sería la pasarela

class AUTOpairing
{
  
  enum PairingStatus {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, };
  
  struct struct_rtc{
    uint32_t code;
    struct_pairing data;
  } ;
  
  static int mensajes_sent;
  static PairingStatus pairingStatus;
  static struct_rtc rtcData;
  static struct_pairing pairingData;
  static bool mensaje_enviado; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static bool terminar; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static unsigned long previousMillis_scanChannel;    // will store last time channel was scanned 
  static unsigned long start_time;  // para controlar el tiempo de escaneo
  static bool esperando;  // esperando mensajes

  static void (*user_callback)(String, String); 

//-----------------------------------------------------------
static int add_byte2message(char byte, char *mensaje, int len)
{
  for(int i=len; i>=0; i--)  // len para copiar caracter fin de cadena
  mensaje[i+1]=mensaje[i];
  mensaje[0]=byte;
  return len+1;
}

//-----------------------------------------------------------
static void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if(debug) Serial.print(macStr);
}


public:

  static unsigned long timeOut;
  static bool debug;
  static bool SettimeOut;
  static uint8_t channel;  // canal para empezar a escanear
  static int segundos_en_deepSleep;   // tiempo dormido en segundos
  static bool usarFLASH;

//-----------------------------------------------------------
static void begin()
  {
    previousMillis_scanChannel=0;
    start_time=millis();
    
    pairingStatus = PAIR_REQUEST;
    
    if(debug) Serial.println();

  
//init check FLASH or RTC MEM
if(usarFLASH) {
  EEPROM.begin(sizeof(struct_pairing)+2);
  // comprobamos si en la flash hay configuracion de emparejamiento
  if(EEPROM.read(0) == MAGIC_BYTE && EEPROM.read(1) == MAGIC_BYTE)
  {
    // tenemos configuración en FLASH
    EEPROM.get(2, pairingData);
    if(debug) Serial.print("Pairing done from FLASH ");
    printMAC(pairingData.macAddr);
    if(debug) Serial.print(" on channel " );
    if(debug) Serial.print(pairingData.channel);    // channel used by the server
    if(debug) Serial.print(" in ");
    if(debug) Serial.print(millis()-start_time);
    if(debug) Serial.println("ms");
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    if(debug) Serial.print(" MY MAC ADDRESS: ");   
    if(debug) Serial.println(WiFi.macAddress());
    wifi_promiscuous_enable(1);
    wifi_set_channel(pairingData.channel);
    wifi_promiscuous_enable(0);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
      if(debug) Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
     
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
    esp_now_register_send_cb(AUTOpairing::OnDataSent);

    pairingData.id = BOARD_ID; 
   
    esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
    pairingStatus = PAIR_PAIRED ;            // set the pairing status
  }
} else { //usar RTC MEM
  // usamos rtc memory
  uint32_t rtcCODE;
  ESP.rtcUserMemoryRead(0,(uint32_t *)&rtcData, sizeof(rtcData));
  if(debug) Serial.print("Magic code on RTC MEM: ");
  if(debug) Serial.print(rtcData.code);
  if(debug) Serial.print(" Correct code: ");
  if(debug) Serial.println((unsigned long)MAGIC_CODE);
  
  if(rtcData.code==MAGIC_CODE)
  {
    //ESP.rtcUserMemoryRead(64, (uint32_t *)&pairingData, sizeof(pairingData));
    memcpy(&pairingData, &(rtcData.data), sizeof(pairingData));
    if(debug) Serial.print("Pairing done from rtc USER MEM ");
    printMAC(pairingData.macAddr);
    if(debug) Serial.print(" on channel " );
    if(debug) Serial.print(pairingData.channel);    // channel used by the server
    if(debug) Serial.print(" in ");
    if(debug) Serial.print(millis()-start_time);
    if(debug) Serial.println("ms");
   
  // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    if(debug) Serial.print(" MY MAC ADDRESS: ");   
    if(debug) Serial.println(WiFi.macAddress());
    wifi_promiscuous_enable(1);
    wifi_set_channel(pairingData.channel);
    wifi_promiscuous_enable(0);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
      if(debug) Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
     
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
    esp_now_register_send_cb(AUTOpairing::OnDataSent);

    pairingData.id = BOARD_ID; 
   
    esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
    pairingStatus = PAIR_PAIRED ;            // set the pairing status
  }
} // end if usarFLASH

}


//-----------------------------------------------------------
// Callback when data is sent
static void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if(debug) Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    if(debug) Serial.println("Delivery success");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
       mensaje_enviado=false;
       if (terminar && !esperando) gotoSleep();
    }
  }
  else{
    if(debug) Serial.println("Delivery fail");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
      //no hemos conseguido hablar con la pasarela emparejada...
      // invalidamos config en flash;
if(usarFLASH) { 
      EEPROM.write(0, NOT_VALID);
      EEPROM.write(1, NOT_VALID);
      EEPROM.commit();
} else {  //usar RTC MEM
      rtcData.code=INVALID_CODE;
      rtcData.data.channel= 2;
      ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
} // end if usarFLASH
      if(debug) Serial.println("FLASH invalidada");
      pairingStatus = PAIR_REQUEST; // volvemos a intentarlo?
      mensaje_enviado=false;
      //delay(100);
      //gotoSleep();
    }
  }
  
}

//-----------------------------------------------------------
// Callback when data is received
static void OnDataRecv(uint8_t * mac, uint8_t *incommingData, uint8_t len) {
  if(debug) Serial.print("Size of message : ");
  if(debug) Serial.print(len);
  if(debug) Serial.print(" from ");
  printMAC(mac);
  if(debug) Serial.println();
  uint8_t type = incommingData[0];
  if(debug) Serial.printf("message type = " BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(type));
  uint8_t i;
  String topic;
  String payload;
  
  switch (type & MASK_MSG_TYPE) {

  case NODATA: 
        esperando = false;
        if(debug) Serial.println("No hay mensajes MQTT"); 
        if (terminar) gotoSleep();
    break;  
    
  case DATA:
        // recibimos mensaje mqtt, trasladarlo al callback() 
        if(debug) Serial.println("Mensaje recibido MQTT"); 
        for(i=0; i<len; i++) if(incommingData[i]=='|') break;
        topic=String(std::string((char*)incommingData,i).c_str());
        payload = String(std::string((char*)(incommingData+i+1),len-i-1).c_str());
        if(user_callback!=NULL) user_callback(topic,payload);
    break;

  case PAIRING:
    memcpy(&pairingData, incommingData, sizeof(pairingData));
    if (pairingData.id == 0) {                // the message comes from server
      if(debug) Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      if(debug) Serial.print(" on channel " );
      if(debug) Serial.print(pairingData.channel);    // channel used by the server
      if(debug) Serial.print(" in ");
      if(debug) Serial.print(millis()-start_time);
      if(debug) Serial.println("ms");
      //esp_now_del_peer(pairingData.macAddr);
      //esp_now_del_peer(mac);
      esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
      pairingStatus = PAIR_PAIRED ;            // set the pairing status
      //guardar en FLASH
if(usarFLASH) { 
      if(debug) Serial.println("Escribimos emparejamiento en FLASH");
      EEPROM.write(0,MAGIC_BYTE);
      EEPROM.write(1,MAGIC_BYTE);
      EEPROM.put(2, pairingData);
      EEPROM.commit();
} else {  //usar RTC MEM
      rtcData.code=MAGIC_CODE;
      memcpy(&(rtcData.data), &pairingData, sizeof(pairingData));
      ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
}
    }
    break;
  }  
}

//void func ( void (*f)(int) );
//-----------------------------------------------------------
static void set_callback( void (*_user_callback)(String, String) ) 
{
  user_callback=_user_callback;
}
  
//-----------------------------------------------------------
static void check_messages()
  {
    mensaje_enviado=true;
    esperando=true;
    timeOut+=500;
    uint8_t mensaje_esp;
    mensaje_esp=CHECK;
    if(debug) Serial.print("Enviando petición de compronación de mensajes... ");
    esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, 1);
  }

//-----------------------------------------------------------
static bool espnow_send_check(char * mensaje, bool fin=true, uint8_t _msgType=DATA)
  {
    esperando=true;
    timeOut+=500;
    return espnow_send(mensaje, fin, _msgType | CHECK);
  }
//-----------------------------------------------------------
static bool espnow_send(char * mensaje, bool fin=true, uint8_t _msgType=DATA)
  {
    if(debug) Serial.printf("message type = "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(_msgType));
    mensaje_enviado=true;
    terminar=fin;
    mensajes_sent++;
    int size = strlen(mensaje);
    if (size> 249)
    {
      if(debug) Serial.print("Longitud del mensaje demasido larga: ");
      if(debug) Serial.println(size);
      return false;
    }
    struct_espnow mensaje_esp;
    mensaje_esp.msgType=_msgType;
    memcpy(mensaje_esp.payload, mensaje, size); 
    if(debug) Serial.print("Longitud del mensaje: ");
    if(debug) Serial.println(size);
    if(debug) Serial.print("ESP-NOW mensaje: ");
    if(debug) Serial.println(mensaje);
    esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, size+1);
    return true;
  }

//-----------------------------------------------------------
static int mensajes_enviados()
{
  return mensajes_sent;
}

static bool emparejado()
{
  return (pairingStatus==PAIR_PAIRED) ;
}

static bool envio_disponible()
{
  return (pairingStatus==PAIR_PAIRED && mensaje_enviado==false && terminar==false) ;
}

//-----------------------------------------------------------
static bool mantener_conexion()
  {
  unsigned long currentMillis;
  if(millis()-start_time > timeOut && SettimeOut)
  {
    if(debug) Serial.println("SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
    gotoSleep();
  }
  
  switch(pairingStatus) {
  case PAIR_REQUEST:
    if(debug) Serial.print("Pairing request on channel "  );
    if(debug) Serial.println(channel);
  
    // clean esp now
    esp_now_deinit();
    WiFi.mode(WIFI_STA);
    // set WiFi channel   
    wifi_promiscuous_enable(1);
    wifi_set_channel(channel);
    wifi_promiscuous_enable(0);
    //WiFi.printDiag(Serial);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
      if(debug) Serial.println("Error initializing ESP-NOW");
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    // set callback routines
    esp_now_register_send_cb(AUTOpairing::OnDataSent);
    esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
    
    // set pairing data to send to the server
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;
    previousMillis_scanChannel = millis();
    // add peer and send request
    if(debug) Serial.println(esp_now_send(broadcastAddressX, (uint8_t *) &pairingData, sizeof(pairingData)));
    pairingStatus = PAIR_REQUESTED;
    break;

  case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis(); 
    if(currentMillis - previousMillis_scanChannel > 100) {
      previousMillis_scanChannel = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > 11) {
        channel = 1x;
      }
      pairingStatus = PAIR_REQUEST; 
    }
    break;

  case PAIR_PAIRED:
   // Serial.println("Paired!");
    break;
  }
  return (pairingStatus==PAIR_PAIRED) ;
} 


//--------------------------------------------------------
static void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  if(debug) Serial.println("Apaga y vamonos");
  ESP.deepSleepInstant(segundos_en_deepSleep * 1000000, RF_NO_CAL);
}

// end of class  
};

// statics:

 AUTOpairing::struct_rtc AUTOpairing::rtcData;
 struct_pairing AUTOpairing::pairingData;
 AUTOpairing::PairingStatus AUTOpairing::pairingStatus = PAIR_REQUEST;
 bool AUTOpairing::mensaje_enviado=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
 bool AUTOpairing::terminar=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
 bool AUTOpairing::esperando=false; 
 bool AUTOpairing::SettimeOut=true; 
 int AUTOpairing::segundos_en_deepSleep = 10;   // tiempo dormido en segundos
 unsigned long AUTOpairing::start_time=0;  // para controlar el tiempo de escaneo
 unsigned long AUTOpairing::previousMillis_scanChannel=0;   
 unsigned long AUTOpairing::timeOut=3000;
 bool AUTOpairing::debug=true;
 bool AUTOpairing::usarFLASH=false;
 uint8_t AUTOpairing::channel = 1;  // canal para empezar a escanear
 int AUTOpairing::mensajes_sent=0; 
 void (*AUTOpairing::user_callback)(String, String)=NULL;

#endif
