///////////////////////////////////////////CODE PROF//////////////////////////////////////////////

/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
   - adapted by A.Combes ISIS 26.1.2025, also inspired by https://wolles-elektronikkiste.de/en/esp-now
*********/
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define BOARD_ID 1

// Wifi Access Point SSID
const char *ssid = "iot";

unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 2000;       // Interval at which to publish sensor readings
unsigned int readingId = 0;

// TODO : change variables below according to project needs
const int vibPin = 26;    // Pin du vibreur
const int btnPin = 25;    // Pin du bouton
bool vibreurEtat = false; // État du vibreur (initialement éteint)

// MAC Address of the sink
// TODO : enter your sink MAC address here
uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0x14, 0x3D, 0x6C};
esp_now_peer_info_t peerInfo;

// ================================================================================================
// DATA STRUCTURES DEFINITION FOR ESPNOW COMMUNICATION
// (Can be modified according to your needs, warning : must be identical on mote and sink)
//
// Create structure to exchange data through ESP-NOW : Mote->Sink
typedef struct struct_mote2sinkMessage
{
  int boardId;
  int readingId;
  int timeTag;
  float data0;
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  float data6;
  bool bool0;
  bool bool1;
  bool bool2;
  bool bool3;
  char text[64];
} struct_mote2sinkMessage;
// Create a struct_message
struct_mote2sinkMessage espNow_moteData;

// Create structure to exchange data through ESP-NOW : Sink->Mote
typedef struct struct_sink2moteMessage
{
  int boardId;
  float data0;
  float data1;
  bool bool0;
  bool bool1;
  bool bool2;
  bool bool3;
  char text[64];
} struct_sink2moteMessage;
struct_sink2moteMessage espNow_incomingMessage;

// ================================================================================================
// Sensor readings :
// TODO : the functions below are examples, adapt to project needs

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ================================================================================================
// Functions for ESP NOW communications
//
// Callback function that will be executed when data is received (sink -> mote)
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from MAC = ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(", length = ");
  Serial.print(len);
  Serial.println(" Bytes");
  memcpy(&espNow_incomingMessage, incomingData, sizeof(espNow_incomingMessage));
  Serial.print(" - boardID : ");
  Serial.println(espNow_incomingMessage.boardId);
  Serial.print(" - data0 : ");
  Serial.println(espNow_incomingMessage.data0);
  Serial.print(" - data1  : ");
  Serial.println(espNow_incomingMessage.data1);
  Serial.print(" - bool0 (vibreur) : ");
  Serial.println(espNow_incomingMessage.bool0);
  Serial.print(" - bool1 : ");
  Serial.println(espNow_incomingMessage.bool1);
  Serial.print(" - bool2 : ");
  Serial.println(espNow_incomingMessage.bool2);
  Serial.print(" - bool3 : ");
  Serial.println(espNow_incomingMessage.bool3);

  // Make a local copy of last received board data
  if (espNow_incomingMessage.boardId != BOARD_ID)
  {
    Serial.println("Message does not correspond to this board Id");
    return;
  }

  ////// MESSAGE RECU PAR LE SINK POUR METTRE LE VIBREUR OFF////

  // Serial.println("Message reçu du Sink");
  // if (espNow_incomingMessage.bool1 == 0)
  // {
  //   Serial.println("Désactive le vibreur maintenant !");
  //   digitalWrite(vibPin, LOW); // Désactive le vibreur
  // }

  // Si l'état du vibreur est "on" ou "off" dans la donnée reçue
  if (espNow_incomingMessage.bool0 == 1)
  {
    Serial.println("Vibreur activé !");
    digitalWrite(vibPin, HIGH); // Active le vibreur
  }
  else
  {
    Serial.println("Vibreur désactivé !");
    digitalWrite(vibPin, LOW);
  }
}

// Callback when data is sent  (mote -> sink)
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks())
  {
    for (uint8_t i = 0; i < n; i++)
    {
      if (!strcmp(ssid, WiFi.SSID(i).c_str()))
      {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

// ================================================================================================
// Setup and Loop functions below
//
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  // TODO : Here perform setup of hardware which is connected to sink (RFID, dht, LEDs, etc.), if needed
  pinMode(vibPin, OUTPUT); // Pin pour le vibreur
  pinMode(btnPin, INPUT);  // Pin pour le bouton

  //-----------------------------------------------------------
  // Settings for Wifi configuration

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  int32_t channel = getWiFiChannel(ssid);
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //-----------------------------------------------------------
  // Settings for ESP NOW configuration
  //
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet and get recv packer info
  esp_now_register_send_cb(espNowOnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(espNowOnDataRecv));

  // Register peer (sink)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

// TODO : Modify code below according to project needs, the following is
// only an example (simulation of temperature and humidity)
void loop()
{
  //   if (digitalRead(btnPin) == HIGH)  // Si le bouton est appuyé
  // {
  //   Serial.println("bouton appuyé");
  //   vibreurEtat = false;  // l'état du vibreur à stop
  //   digitalWrite(vibPin, vibreurEtat);  // Allume ou éteint le vibreur
  //   espNow_incomingMessage.bool0 == 0;
  //   Serial.println(vibreurEtat ? "Vibreur ON" : "Vibreur OFF");

  // }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    espNow_moteData = {};
    boolean alarme = false;
    // Set values to send
    espNow_moteData.boardId = BOARD_ID;
    espNow_moteData.readingId = readingId++;
    espNow_moteData.timeTag = currentMillis;
    if (digitalRead(btnPin) == HIGH) // Si le bouton est appuyé
    {
      // digitalWrite(vibPin, LOW);
      espNow_moteData.bool1 = 1;
      Serial.println("bouton appuyé");

      // espNow_moteData.bool1 = 1;
    }
    else
    {
      espNow_moteData.bool1 = 0;
      // espNow_moteData.bool1 = 0;
      Serial.println("bouton pas appuyé");
    }

    espNow_moteData.readingId = readingId++;
    char textMsg[] = "Hi Sink, here's my data for you: ";
    memcpy(&espNow_moteData.text, textMsg, sizeof(textMsg));

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espNow_moteData, sizeof(espNow_moteData));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }

  }
}
