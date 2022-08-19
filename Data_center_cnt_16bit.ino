#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <eddystone.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>

#include "time.h"
#include "math.h"
#include "esp_wpa2.h"
#include <String.h>
#include "EEPROM.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

#include "esp_system.h"

#define SSID_addr   0
#define USER_addr   32
#define PASS_addr   64
#define ID_addr 96
#define flag_select_addr  116 //96
#define check_get_addr 118 //98
//#define token_addr 120 //100

//#define BUFF_addr   64
#define EEPROM_SIZE 4096
#define addr_size   32
#define addr_size_id 10
#define RXD2 16
#define TXD2 17
/*
  #define mqtt_name "esp32-0001"
  const char * topic = "IOT/lab/test-queue/"; // topic ชื่อ /server
  const char* mqtt_server = "150.109.185.202";
  const int mqtt_port = 1883;
  const char* mqtt_user = "station-mesh0010";
  const char* mqtt_password = "rabbITMQ_Prototype_Stack";
  String ID_station_hardcode = "10";
*/

const char * topic = "IOT/lab/station0025/"; // topic ชื่อ /server
#define mqtt_server "124.156.247.198" // server
#define mqtt_port 1883 // เลข port
#define mqtt_user "station0025" // user
#define mqtt_password "8G+<*xV(C""\"""YjUvJw" // password
#define mqtt_name "esp32-0025"
String ID_station_hardcode = "25";

WiFiClient StationClient;
PubSubClient esp32(StationClient);

Scheduler get_uart;
void uart250ms();
Task data_uart(100, TASK_FOREVER, &uart250ms);

static QueueHandle_t queue_send;

std::string NAMESPACE;
std::string INSTANCE;

unsigned long time_get = 0;

const int LED_G = 32;
const int LED_R = 33;
const int LED_B = 25;
const int buzzer = 15;
const int button = 26;         //gpio to use to trigger delay //26new0old
const int wdtTimeout = 2000;  //2000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

int instance = 0;
int wifi_status = 0;
int wificonnect_count = 0;
//String auth_str;
//String auth_acc;
//String accToken;
char beacon[4096];

String nrf_data;
String nrf_data_queue;
int size_data = 0;
int num_uart = 0;
int flag_count = 0;

//int check_token = 0;
int old_num_node = 0;
int round_scan = 0;
String des_id = "000000";

const char* ntpServer           = "pool.ntp.org";
const long  gmtOffset_sec       = 6 * 3600;
const int   daylightOffset_sec  = 3600;

char tm_put[100];

bool activate_flag = false;
bool SSID_flag = false;
bool PASS_flag = false;

WiFiMulti WiFiMulti;

static uint8_t SSID_Wifi[32];
static uint8_t USER_Wifi[32];
static uint8_t PASS_Wifi[32];
//static uint8_t refresh_token[50];
uint8_t NODE_ID[10];
char LEVEL_NODE[3];
//const char* SSID_Wifi[32];
//const char* USER_Wifi[32];
//const char* PASS_Wifi[32];
static bool hardware_reset = false;

//int flag_select = 0;
int flag = 0;

int near_network = 0;
int level_boardcast = 0;

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pSSIDCharacteristic    = NULL;
BLECharacteristic* pUSERCharacteristic    = NULL;
BLECharacteristic* pPASSCharacteristic    = NULL;
BLECharacteristic* pMACACharacteristic    = NULL;
BLECharacteristic* pRESTCharacteristic    = NULL;
BLECharacteristic* pNODECharacteristic    = NULL;

//String url_send = "http://178.128.96.178/api/device/" + WiFi.macAddress();
//const char *auth;
//String auth;
String id_station;

unsigned long time_boardcast = 0;
unsigned long time_deadnode = 0;
unsigned long time_id = 0;

BLEScan* pBLEScan;
BLEAdvertising *pAdvertising;
//BLEServer *pServer;

//static BLERemoteCharacteristic* pRemoteCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint16_t beconUUID = 0xFEAA;
uint8_t sookjaiUUID[10] = {0x41, 0x74, 0x61, 0x70, 0x79, 0x50, 0x65, 0x6E, 0x48, 0x41};
int scanTime = 1; //In seconds

#define SERVICE_UUID                "39d80010-5c32-434c-86bd-a30a55b93583"
#define SSID_CHARACTERISTIC_UUID    "39d80011-5c32-434c-86bd-a30a55b93583"
#define USER_CHARACTERISTIC_UUID    "39d80012-5c32-434c-86bd-a30a55b93583"
#define PASS_CHARACTERISTIC_UUID    "39d80013-5c32-434c-86bd-a30a55b93583"
#define MACA_CHARACTERISTIC_UUID    "39d80014-5c32-434c-86bd-a30a55b93583"
#define REST_CHARACTERISTIC_UUID    "39d80015-5c32-434c-86bd-a30a55b93583"
#define NODE_CHARACTERISTIC_UUID    "39d80016-5c32-434c-86bd-a30a55b93583"

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

void IRAM_ATTR isrfbutton();

const char* rootCACertificate = \
                                "-----BEGIN CERTIFICATE-----\n" \
                                "MIIE5TCCBIqgAwIBAgIQC+1+syL8NOkRg4WZsmTR/DAKBggqhkjOPQQDAjBvMQsw\n" \
                                "CQYDVQQGEwJVUzELMAkGA1UECBMCQ0ExFjAUBgNVBAcTDVNhbiBGcmFuY2lzY28x\n" \
                                "GTAXBgNVBAoTEENsb3VkRmxhcmUsIEluYy4xIDAeBgNVBAMTF0Nsb3VkRmxhcmUg\n" \
                                "SW5jIEVDQyBDQS0yMB4XDTE5MTAxNzAwMDAwMFoXDTIwMTAwOTEyMDAwMFowbTEL\n" \
                                "MAkGA1UEBhMCVVMxCzAJBgNVBAgTAkNBMRYwFAYDVQQHEw1TYW4gRnJhbmNpc2Nv\n" \
                                "MRkwFwYDVQQKExBDbG91ZGZsYXJlLCBJbmMuMR4wHAYDVQQDExVzbmkuY2xvdWRm\n" \
                                "bGFyZXNzbC5jb20wWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAASw+HWUuQtOnYIa\n" \
                                "4jZPvlGYxqc9l8+06enb0hO6FYS+QD2hJPjixCPjKMGOCaSUkyelI0i6KcX5fdEC\n" \
                                "FLmAWDNIo4IDCDCCAwQwHwYDVR0jBBgwFoAUPnQtH89FdQR+P8Cihz5MQ4NRE8Yw\n" \
                                "HQYDVR0OBBYEFEyrw/0pFP04Z5AsRFSa5zdg33ZPMDwGA1UdEQQ1MDOCFXNuaS5j\n" \
                                "bG91ZGZsYXJlc3NsLmNvbYILYXRhcHkuY28udGiCDSouYXRhcHkuY28udGgwDgYD\n" \
                                "VR0PAQH/BAQDAgeAMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjB5BgNV\n" \
                                "HR8EcjBwMDagNKAyhjBodHRwOi8vY3JsMy5kaWdpY2VydC5jb20vQ2xvdWRGbGFy\n" \
                                "ZUluY0VDQ0NBMi5jcmwwNqA0oDKGMGh0dHA6Ly9jcmw0LmRpZ2ljZXJ0LmNvbS9D\n" \
                                "bG91ZEZsYXJlSW5jRUNDQ0EyLmNybDBMBgNVHSAERTBDMDcGCWCGSAGG/WwBATAq\n" \
                                "MCgGCCsGAQUFBwIBFhxodHRwczovL3d3dy5kaWdpY2VydC5jb20vQ1BTMAgGBmeB\n" \
                                "DAECAjB2BggrBgEFBQcBAQRqMGgwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRp\n" \
                                "Z2ljZXJ0LmNvbTBABggrBgEFBQcwAoY0aHR0cDovL2NhY2VydHMuZGlnaWNlcnQu\n" \
                                "Y29tL0Nsb3VkRmxhcmVJbmNFQ0NDQS0yLmNydDAMBgNVHRMBAf8EAjAAMIIBBAYK\n" \
                                "KwYBBAHWeQIEAgSB9QSB8gDwAHYAu9nfvB+KcbWTlCOXqpJ7RzhXlQqrUugakJZk\n" \
                                "No4e0YUAAAFt2fOERwAABAMARzBFAiBaRx21vJRcnTZAi6jWKUeCkCQN0DKsSphJ\n" \
                                "a6mTJHXqGgIhALW4EeN/9N8/N9jjMcoxWg8Y1PZcJUQssMGa3QoNRKfeAHYAXqdz\n" \
                                "+d9WwOe1Nkh90EngMnqRmgyEoRIShBh1loFxRVgAAAFt2fODnwAABAMARzBFAiBt\n" \
                                "m9XUSuct6oGrDlUijsFSsmrfniP9EGKVWr8eP7y13gIhAL207IfzxwbNrdTxKHW/\n" \
                                "ZLMky3h+5VOafdUgiOvr+RVjMAoGCCqGSM49BAMCA0kAMEYCIQCSGBrFaM17X3Qx\n" \
                                "RwiOQe/Zio+OTGDFgHbhPdtSMbWHqwIhAPzYu1JsqQB2oNipQ0A3EU9NL8B5ASM4\n" \
                                "0v8CeH2xh5+b\n" \
                                "-----END CERTIFICATE-----\n";

void IRAM_ATTR resetModule() {
  hardware_reset = true;
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
}

void IRAM_ATTR isrrbutton() {
  timerAlarmDisable(timer);
  timer = NULL;
  attachInterrupt(button, isrfbutton, FALLING);
}

void IRAM_ATTR isrfbutton() {
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);
  attachInterrupt(button, isrrbutton, RISING);
}

char* printLocalTime(char* tm_put) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return NULL;
  }
  time_t current = time(0);
  //Serial.print("Unixtime = ");
  //Serial.println(current);
  tm*localtime();
  //sprintf(tm_put, "%d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  sprintf(tm_put, "%d" , current);
  return tm_put;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
class MySSIDCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      memset(SSID_Wifi, 0, sizeof(SSID_Wifi));
      memset(USER_Wifi, 0, sizeof(USER_Wifi));
      EEPROM.write(flag_select_addr, 0);
      EEPROM.commit();
      //    Serial.print("SSID: ");
      //Serial.println(SSID_flag);

      uint8_t* ssid_wifi = pCharacteristic->getData();
      for (int j = 0; j < addr_size; j++) {
        if ((ssid_wifi[j] != 0x00) && (ssid_wifi[j] != 0xFF)) {
          SSID_Wifi[j] = ssid_wifi[j];
        }
      }

      //SSID_Wifi = pCharacteristic->getData();
      //Serial.printf("%s\n", SSID_Wifi);
      //WiFi.mode(WIFI_STA);
      //WiFi.disconnect();
      int n = WiFi.scanNetworks();
      if (n < 0) {
        esp_restart();
      }
      Serial.println("scan done");
      if (n == 0) {
        Serial.println("no networks found");
      } else  {
        Serial.println("networks found");
        for (int i = 0; i < n; ++i) {
          int m = WiFi.SSID(i).length();
          char temp_array[m + 1];
          strcpy(temp_array, WiFi.SSID(i).c_str()); // copy temp_array ไปยัง WiFi.SSID(i).c_str()
          if (memcmp(temp_array, SSID_Wifi, m + 1) == 0) {
            Serial.print("SSID: ");
            Serial.printf("%s", SSID_Wifi);
            Serial.print("\t\t : SSID is Correct");
            Serial.print("\t\t\t ,Write SSID :  ");
            Serial.printf("%s", SSID_Wifi);
            Serial.println(" in EEPROM");
            notify();
            for (int k = 0; k < addr_size; k++) {
              if ((SSID_Wifi[k] == 0x00) || (SSID_Wifi[k] == 0xFF)) {
                break;
              } else {
                //EEPROM.write((k + SSID_addr), SSID_Wifi[k]);
                //Serial.print(".");
                //break;
              }
            }
            SSID_flag = true;
            return;
          }
        }
        Serial.print("SSID :  ");
        Serial.printf("%s", SSID_Wifi);
        Serial.println(" incorrect");
        Serial.println("try again");
        notify2();
        /*
          for (int i = 0; i < addr_size; i++) {
          SSID_Wifi[i] = 0;
          ssid_wifi[i] = 0;
          }
        */
        // Return SSID not match.
      }
    }
};

class MyUSERCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      memset(USER_Wifi, 0, sizeof(USER_Wifi));
      uint8_t* user_wifi = pCharacteristic->getData();
      for (int j = 0; j < addr_size; j++) {
        if ((user_wifi[j] != 0x00) && (user_wifi[j] != 0xFF)) {
          USER_Wifi[j] = user_wifi[j];
        }
      }
      /*
        for (int k = 0; k < addr_size; k++) {
        if ((USER_Wifi[k] == 0x00) || (USER_Wifi[k] == 0xFF)) {
          EEPROM.write(flag_select_addr, 0);
          //flag = 0;
          break;
        } else {
          //Serial.println("write user");
          //flag = 1;
          //EEPROM.write((k + USER_addr), USER_Wifi[k]);
          //EEPROM.write(flag_select_addr, 1);

          //Serial.print(".");
          //break;
        }
        }*/
      EEPROM.write(flag_select_addr, 1);
      EEPROM.commit();
      //Serial.printf("flag : %d\n", flag);
      Serial.printf(" USER :  %s\n", USER_Wifi);
    }
};

class MyPASSCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      memset(PASS_Wifi, 0, sizeof(PASS_Wifi));
      uint8_t* pass_wifi = pCharacteristic->getData();
      for (int j = 0; j < addr_size; j++) {
        if ((pass_wifi[j] != 0x00) && (pass_wifi[j] != 0xFF)) {
          PASS_Wifi[j] = pass_wifi[j];
        }
      }

      /*
        for (int j = 0; j < addr_size; j++) {
        if ((EEPROM.read(j + USER_addr) != 0x00) && (EEPROM.read(j + USER_addr) != 0xFF)) {
          USER_Wifi[j] = EEPROM.read(j + USER_addr);
          //Serial.println("USER");
          flag = 1;
        }
        else if ((EEPROM.read(j + USER_addr) == 0x00) && (EEPROM.read(j + USER_addr) == 0xFF)) {
          flag = 2;
        }
        }*/

      WiFi.mode(WIFI_STA);
      if (EEPROM.read(flag_select_addr) == 1) {
        Serial.println("Try connect to...WPA ENTERPRISE");
        esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)USER_Wifi, strlen((const char*)USER_Wifi)); //provide identity
        esp_wifi_sta_wpa2_ent_set_username((uint8_t *)USER_Wifi, strlen((const char*)USER_Wifi)); //provide username --> identity and username is same
        esp_wifi_sta_wpa2_ent_set_password((uint8_t *)PASS_Wifi, strlen((const char*)PASS_Wifi)); //provide password
        esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
        esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
        WiFi.begin((const char*)SSID_Wifi); //connect to wifi Enterpise
      } else {
        Serial.println("Try connect to...WPA PERSONAL");
        WiFi.begin((const char*)SSID_Wifi, (const char*)PASS_Wifi); //connect to wifi Personal
      }

      for (int i = 0; i < 10; i++) {
        Serial.print(".");
        //WiFi.begin((const char*)SSID_Wifi, (const char*)PASS_Wifi);
        //delay(500);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.print("Password: ");
          Serial.printf("%s", PASS_Wifi);
          Serial.print("\t : Password is Correct");
          Serial.print("\t\t\t ,Write Password :  ");
          Serial.printf("%s", PASS_Wifi);
          Serial.println(" in EEPROM");
          notify3();

          for (int k = 0; k < addr_size; k++) {
            if (((PASS_Wifi[k] == 0x00) || (PASS_Wifi[k] == 0xFF)) && ((SSID_Wifi[k] == 0x00) || (SSID_Wifi[k] == 0xFF)) && ((USER_Wifi[k] == 0x00) || (USER_Wifi[k] == 0xFF))) {
              break;
            } else {
              if (EEPROM.read(flag_select_addr) == 1) {
                EEPROM.write((k + SSID_addr), SSID_Wifi[k]);
                EEPROM.write((k + USER_addr), USER_Wifi[k]);
                EEPROM.write((k + PASS_addr), PASS_Wifi[k]);
              } else {
                EEPROM.write((k + SSID_addr), SSID_Wifi[k]);
                EEPROM.write((k + PASS_addr), PASS_Wifi[k]);
              }
            }
          }
          EEPROM.commit();
          EEPROM.end();
          Serial.println("Success");
          Serial.println("connected");
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());
          PASS_flag = true;
          return;
        }
        delay(500);
      }
      notify4();
      Serial.print("CAN'T connect to SSID: ");
      Serial.printf("%s\n", SSID_Wifi);
      /*
        for (int i = 0; i < addr_size; i++) {
        PASS_Wifi[i] = 0;
        pass_wifi[i] = 0;
        }
      */
    }
};

class MyMACACharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharacteristic) {
      int n = WiFi.macAddress().length();
      char temp_array[n + 1];
      strcpy(temp_array, WiFi.macAddress().c_str());
      pCharacteristic->setValue((uint8_t*)temp_array, n + 1);
    }
};

class MyRESTCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      activate_flag = true;
    }
};

class MyNODECharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      uint8_t* node_id = pCharacteristic->getData();
      memset(NODE_ID, 0, sizeof(node_id));
      for (int i = 0; i < addr_size_id; i++) {
        if ((node_id[i] != 0x00) && (node_id[i] != 0xFF)) {
          NODE_ID[i] = node_id[i];
        }
      }
      Serial.printf(" NODE_ID :  %s\n", NODE_ID);

      EEPROM.begin(EEPROM_SIZE);
      for (int k = 0; k < addr_size_id; k++) {
        if ((NODE_ID[k] == 0x00) || (NODE_ID[k] == 0xFF)) {
          break;
        } else {
          Serial.printf("write ID_addr %c ID_addr %d \r\n", NODE_ID[k], k + ID_addr);
          EEPROM.write((k + ID_addr), NODE_ID[k]);
        }
      }
      EEPROM.commit();
      EEPROM.end();
    }

};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.getServiceDataUUID().equals(BLEUUID(beconUUID)) == true) {
        //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        uint8_t* payload;
        payload = advertisedDevice.getPayload();

        if ( (payload[13] == 0x41) && (payload[14] == 0x50) && (payload[15] == 0x53) && (payload[16] == 0x4E)) {
          //Serial.println("Station Node");

          time_deadnode = millis();
          if (near_network != 1) {
            near_network = 2;

            /*
              String network_namespace;
              for (int i = 0; i < 10; i++) {

              network_namespace += String(payload[13 + i], HEX);

              }
              Serial.println(network_namespace);
            */

            int level_from_node = payload[17];

            if (round_scan == 0) {

              old_num_node = level_from_node;
              level_boardcast = level_from_node + 1;
              des_id = String(payload[18], HEX) + String(payload[19], HEX) + String(payload[20], HEX);

            } else {
              if (old_num_node <= level_from_node) {

                level_boardcast = old_num_node + 1;

              } else {

                old_num_node = level_from_node;
                level_boardcast = level_from_node + 1;
                des_id = String(payload[18], HEX) + String(payload[19], HEX) + String(payload[20], HEX);
              }
            }

            //int level_boardcast = payload[17] + 1;
            //String level_node = String(level_boardcast, HEX);
            sprintf(LEVEL_NODE, "%02x", level_boardcast);
            //Serial.println(LEVEL_NODE);
          }
          /*
            int inst_length = instance.length();
            char namespace_c[inst_length + 1];
            strcpy(namespace_c, instance.c_str());
            NAMESPACE.append(namespace_c);
            std::cout << NAMESPACE << std::endl;
            NAMESPACE.clear();
          */
        }

        if ((payload[13] == 0x41) && (payload[14] == 0x50) && (payload[15] == 0x53) && (payload[16] == 0x47)) {
          //Serial.println("Station Gateway");

          time_deadnode = millis();
          des_id = String(payload[18], HEX) + String(payload[19], HEX) + String(payload[20], HEX);
          //Serial.print("des_id: ");
          //Serial.println(des_id);

          //String mode_node = "4150534E00";
          near_network = 1;
        }
        round_scan++;
      }
    }
};

void setBeacon(BLEAdvertising* pAdvertising) {
  EddystoneUid uid(NAMESPACE, INSTANCE);
  BLEAdvertisementData adData;
  uid.compose(adData);
  pAdvertising->setAdvertisementData(adData);
}

void StartBeacon() {
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(250);
  pBLEScan->setWindow(250);  // less or equal setInterval value
}

void StartGATTBLE() {
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSSIDCharacteristic   = pService->createCharacteristic(
                            SSID_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                            //BLECharacteristic::PROPERTY_INDICATE
                          );

  pUSERCharacteristic   = pService->createCharacteristic(
                            USER_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                            //BLECharacteristic::PROPERTY_INDICATE
                          );

  pPASSCharacteristic   = pService->createCharacteristic(
                            PASS_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                            //BLECharacteristic::PROPERTY_INDICATE
                          );

  pMACACharacteristic   = pService->createCharacteristic(
                            MACA_CHARACTERISTIC_UUID,
                            //BLECharacteristic::PROPERTY_NOTIFY |
                            //BLECharacteristic::PROPERTY_INDICATE |
                            BLECharacteristic::PROPERTY_READ

                          );

  pRESTCharacteristic   = pService->createCharacteristic(
                            REST_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_WRITE
                          );

  /*pNODECharacteristic   = pService->createCharacteristic(
                            NODE_CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_WRITE
                            //BLECharacteristic::PROPERTY_INDICATE
                          );*/


  // Create a Characteristic Callbacks
  pSSIDCharacteristic->setCallbacks(new MySSIDCharacteristicCallbacks());
  pUSERCharacteristic->setCallbacks(new MyUSERCharacteristicCallbacks());
  pPASSCharacteristic->setCallbacks(new MyPASSCharacteristicCallbacks());
  pMACACharacteristic->setCallbacks(new MyMACACharacteristicCallbacks());
  pRESTCharacteristic->setCallbacks(new MyRESTCharacteristicCallbacks());
  //pNODECharacteristic->setCallbacks(new MyNODECharacteristicCallbacks());



  // Create a BLE Descriptor
  pSSIDCharacteristic->addDescriptor(new BLE2902());
  pUSERCharacteristic->addDescriptor(new BLE2902());
  pPASSCharacteristic->addDescriptor(new BLE2902());
  //pMACACharacteristic->addDescriptor(new BLE2902());
  //pNODECharacteristic->addDescriptor(new BLE2902());
  //pRESTCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  //Start advertising
  //BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a Client Connect to SOOKJAI ...");
}

void notify() {
  Serial.println("correct");
  pSSIDCharacteristic->setValue("Connect to SSID");
  pSSIDCharacteristic->notify();
  //return;
}

void notify2() {
  Serial.println("incorrect");
  pSSIDCharacteristic->setValue("SSID Incorrect");
  pSSIDCharacteristic->notify();
  //return;
}

void notify3() {
  Serial.println("Connect to WIFI success");
  pPASSCharacteristic->setValue("connect to WIFI success");
  pPASSCharacteristic->notify();
  //return;
}

void notify4() {
  Serial.println("not correct");
  pPASSCharacteristic->setValue("Can't connect");
  pPASSCharacteristic->notify();
  //return;
}

// Have SSID and Password from first configuration.
void connectToWifi() {

  //digitalWrite(LED_R, LOW);
  //digitalWrite(LED_G, LOW);
  //digitalWrite(LED_B, HIGH);

  for (int j = 0; j < addr_size; j++) {
    if ((EEPROM.read(j + SSID_addr) != 0x00) && (EEPROM.read(j + SSID_addr) != 0xFF)) {
      SSID_Wifi[j] = EEPROM.read(j + SSID_addr);
      //Serial.println("SSID");
    }
    if ((EEPROM.read(j + PASS_addr) != 0x00) && (EEPROM.read(j + PASS_addr) != 0xFF)) {
      PASS_Wifi[j] = EEPROM.read(j + PASS_addr);
      //Serial.println("PASS");
    }
    if ((EEPROM.read(j + USER_addr) != 0x00) && (EEPROM.read(j + USER_addr) != 0xFF)) {
      USER_Wifi[j] = EEPROM.read(j + USER_addr);
      //Serial.println("USER");
      flag = 1;
    }
    if ((EEPROM.read(j + USER_addr) == 0x00) && (EEPROM.read(j + USER_addr) == 0xFF)) {
      USER_Wifi[j] = EEPROM.read(j + USER_addr);
      //Serial.println("USER");
      flag = 0;
    }
  }
  EEPROM.end();

  if (flag == 1)  {
    Serial.print("\r\nSSID: ");
    Serial.printf("%s", SSID_Wifi);
    Serial.print("\r\nUSER: ");
    Serial.printf("%s", USER_Wifi);
    Serial.print("\r\nPassword: ");
    Serial.printf("%s\n", PASS_Wifi);
  } else  {
    Serial.print("\r\nSSID: ");
    Serial.printf("%s", SSID_Wifi);
    Serial.print("\r\nPassword: ");
    Serial.printf("%s\n", PASS_Wifi);
  }
  //int flag = EEPROM.read(flag_select_addr);
  //Serial.println(flag);
  //Serial.println(EEPROM.read(flag_select_addr));

  WiFi.mode(WIFI_STA);
  if (flag == 1) {
    Serial.println("Try connect to...WPA ENTERPRISE");
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)USER_Wifi, strlen((const char*)USER_Wifi)); //provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)USER_Wifi, strlen((const char*)USER_Wifi)); //provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)PASS_Wifi, strlen((const char*)PASS_Wifi)); //provide password
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
    esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
    WiFi.begin((const char*)SSID_Wifi); //connect to wifi

  } else {
    Serial.println("Try connect to...WPA PERSONAL");
    WiFi.begin((const char*)SSID_Wifi, (const char*)PASS_Wifi);
  }

  while (WiFi.status() != WL_CONNECTED) {

    if (hardware_reset) {
      reset();
    }

    //WiFi.begin((const char*)SSID_Wifi, (const char*)PASS_Wifi);
    if (flag == 0) {
      Serial.print("\r\nSSID: ");
      Serial.printf("%s", SSID_Wifi);
      Serial.print("\r\nPassword: ");
      Serial.printf("%s\n", PASS_Wifi);
    } else {
      Serial.print("\r\nSSID: ");
      Serial.printf("%s", SSID_Wifi);
      Serial.print("\r\nUSER: ");
      Serial.printf("%s", USER_Wifi);
      Serial.print("\r\nPassword: ");
      Serial.printf("%s\n", PASS_Wifi);
    }
    if (wificonnect_count == 10 ) {
      //wificonnect_count = 0;
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
      esp_restart();
      //break;
    }
    wificonnect_count++;
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifi_status = 1;
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    delay(100);
    //for (int blinky_i = 0; blinky_i < 5; blinky_i++) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    /*delay(500);
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_B, LOW);
      delay(500);
      }
    */
    wificonnect_count = 0;
  }
}

void EEPROMInit() {
  EEPROM.begin(EEPROM_SIZE);
  /*Serial.println("hello");
    Check EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
      Serial.print(byte(EEPROM.read(i))); Serial.print(" ");
    }*/
  if (EEPROM.read(SSID_addr) != 0xFF) { //check EEPROM ว่ามีข้อมูลหรือไม่
    Serial.println("Start......");

    // Not initial
    connectToWifi();
    return;
  }

  // Initialization
  StartGATTBLE();
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  while (!activate_flag) {
    if (hardware_reset) {
      reset();
    }
    // notify changed value
    if (deviceConnected) {
      delay(10);  // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      if (hardware_reset) {
        reset();
      }
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
      Serial.print("pairing...");
    }
  }
  uint16_t ConnId = pServer->getConnId();
  pServer->disconnect(ConnId);
  pServer->removeService(pService);
  pServer = NULL;
  delay(1000);

  for (int b = 0; b < 3; b++) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    delay(500);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
    delay(500);
  }
  esp_restart();
}

void WiFiStatus(void * parameter) {       //check status wifi every 500 ms.

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {

      Serial.println("WiFi not connect");

      //digitalWrite(LED_R, LOW);
      //digitalWrite(LED_G, LOW);
      //digitalWrite(LED_B, HIGH);

      wifi_status = 0;

      EEPROM.begin(EEPROM_SIZE);
      connectToWifi();

    } else {

      //Serial.println("WiFi  connect");
      //digitalWrite(LED_R, LOW);
      //digitalWrite(LED_G, HIGH);
      //digitalWrite(LED_B, LOW);
      wifi_status = 1;
    }
    delay(5000);
  }
}

//Get refresh_token for get access_token
/*
  void Get_RefreshToken() {

  String url_get  = "http://178.128.96.178/api/device/getottoken/" + WiFi.macAddress();

  HTTPClient http_id;                                                                       //HTTP POST
  http_id.begin(url_get);
  http_id.addHeader("Content-Type", "application/json");
  int httpCode_id = http_id.POST("");

  if (httpCode_id == 200) {

    String id = http_id.getString();
    StaticJsonBuffer<200>jsonBufferRoot;
    JsonObject& root = jsonBufferRoot.parseObject(id);                                      //Get data จาก server
    //Serial.println(id);

    if (!root.success()) {                                                                  //check status get data
      Serial.println("parseObject() failed");
      esp_restart();

    } else {
      Serial.println("Get token...success");
      String sensor = root["refToken"];   //แปลงค่าที่ได้จาก server เป็น Token ในการส่งข้อมูล
      //Serial.printf("refToken: %s\n", sensor);

      char auth[200];
      sensor.toCharArray(auth, sizeof(auth));

      EEPROM.begin(EEPROM_SIZE);
      EEPROM.write(check_get_addr, 1);
      for (int i = 0; i < sizeof(auth); i++) {
        EEPROM.write(i + token_addr, auth[i]);
      }
      EEPROM.commit();
      EEPROM.end();
      jsonBufferRoot.clear();
    }

  } else {
    Serial.printf("[HTTPS] POST... failed, error:  %s [code: %d]\n", http_id.errorToString(httpCode_id).c_str(), httpCode_id);
    delay(500);
    esp_restart();
  }
  http_id.end();
  delay(100);
  }

  //Get access_token for insert data to database
  void Get_token() {

  String url_acc  = "http://178.128.96.178/api/device/reftoken";
  auth_str = (char*)refresh_token;
  const char *refresh = auth_str.c_str();

  HTTPClient http_acc;                                                                       //HTTP POST
  http_acc.begin(url_acc);
  http_acc.addHeader("Content-Type", "application/json");
  http_acc.setAuthorization(refresh);
  int httpCode_acc = http_acc.POST("");

  if (httpCode_acc == 200) {

    String acc = http_acc.getString();
    StaticJsonBuffer<500>jsonBufferRootacc;
    JsonObject& rootacc = jsonBufferRootacc.parseObject(acc);                                      //Get data จาก server
    //Serial.print("acc:");
    //Serial.println(acc);

    if (!rootacc.success()) {                                                                  //check status get data
      Serial.println("parseObject() failed");
      esp_restart();

    } else {
      Serial.println("Get ACC token...success\n");
      String access = rootacc["accessToken"];                                                  //แปลงค่าที่ได้จาก server เป็น Token ในการส่งข้อมูล
      //Serial.printf("accessToken: %s\n", access);
      //Serial.println(access);
      accToken = access;
      jsonBufferRootacc.clear();

      check_token = 1;
    }
  } else {
    Serial.printf("[HTTPS] POST... failed, error:  %s [code: %d]\n", http_acc.errorToString(httpCode_acc).c_str(), httpCode_acc);
    delay(500);
    esp_restart();
  }
  http_acc.end();
  delay(100);
  }
*/
void callback(char* topic, byte * payload, unsigned int length) {
  //delay(100);
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
}

//mqtt publish
void send_mqtt(String data_mqtt) {
  if (!esp32.connected()) {
    while (!esp32.connected())
    {
      //Serial.println("reconnect");
      esp32.connect(mqtt_name, mqtt_user, mqtt_password);
      if (hardware_reset) {
        reset();
      }
      delay(100);
    }
  } else {
    //Serial.println(data_mqtt);
    char DATA_SEND[data_mqtt.length() + 1];
    data_mqtt.toCharArray(DATA_SEND, sizeof(DATA_SEND));
    esp32.publish(topic, DATA_SEND);
    esp32.loop();
    //Serial.printf("\r millis after: %lu \r\n", millis());
  }
  //Serial.println(data_mqtt);
  //char DATA_SEND[128];
  //data_mqtt.toCharArray(DATA_SEND, sizeof(DATA_SEND));
  //client.publish(topic, DATA_SEND);
  //client.loop();

  /*
    while (!client.connected()) {
    if (hardware_reset) {
      reset();
    }
    client.connect(mqtt_name, mqtt_user, mqtt_password);
    //Serial.print("failed, rc=");
    //Serial.println(client.state());
    delay(100);
    }
    if (client.connected()) {
    //client.subscribe(topic); //ชื่อ topic ที่ต้องการติดตาม
    //Serial.println("MQTT Connected");
    Serial.println(data_mqtt);
    char DATA_SEND[128];
    data_mqtt.toCharArray(DATA_SEND, sizeof(DATA_SEND));
    client.publish(topic, DATA_SEND);
    }*/
}

int invert_varience(String data_vars) {
  //Serial.println(data_vars.length());
  char data_var[data_vars.length() + 1];
  data_vars.toCharArray(data_var, sizeof(data_var));

  uint8_t check_msb = 0;
  char var_1 = data_vars[0];
  if (('0' <= var_1) && (var_1 <= '9'))
    check_msb = (var_1 - '0' );
  else if (('A' <= var_1) && (var_1 <= 'F'))
    check_msb = (var_1 - 'A' + 10) ;
  else if (('a' <= var_1) && (var_1 <= 'f'))
    check_msb = (var_1 - 'a' + 10);


  if ((check_msb >> 3) == 1) {
    int value_vars = 0;

    for (int i = 0; i < data_vars.length(); i++) {
      //check_msb = 0;
      char var = data_vars[i];

      if (('0' <= var) && (var <= '9'))
        check_msb = (var  - '0' );
      else if (('A' <= var) && (var <= 'F'))
        check_msb = (var - 'A' + 10) ;
      else if (('a' <= var) && (var <= 'f'))
        check_msb = (var - 'a' + 10);

      int x = 15;
      if (i == 0)
        //value_vars += (check_msb & 7) * pow(16,  i);
        //value_vars += (check_msb & 7) * pow(16, ((data_vars.length() - 1) - i));
        value_vars += ((check_msb ^ x) * (pow(16, ((data_vars.length() - 1) - i))));
      else
        //value_vars += check_msb * pow(16,  i);
        value_vars += ((check_msb ^ x) * (pow(16, ((data_vars.length() - 1) - i))));
    }
    //Serial.println(~value_vars);
    return ~value_vars;

  } else if ((check_msb >> 3) == 0) {
    int value_vars = 0;

    for (int i = 0; i < data_vars.length(); i++) {
      //check_msb = 0;
      char var = data_vars[i];

      if (('0' <= var) && (var <= '9'))
        check_msb = (var - '0' );
      else if (('A' <= var) && (var <= 'F'))
        check_msb = (var - 'A' + 10) ;
      else if (('a' <= var) && (var <= 'f'))
        check_msb = (var - 'a' + 10);

      if (i == 0)
        //value_vars += (check_msb & 15) * pow(16, ((data_vars.length() - 1) - i));
        value_vars += (check_msb ) * pow(16, ((data_vars.length() - 1) - i));
      else
        value_vars += (check_msb) * pow(16, ((data_vars.length() - 1) - i));
    }
    //Serial.println(value_vars);
    return value_vars;
  }
}

int16_t hex2int(const char *hex)
{
  int16_t value;  // unsigned to avoid signed overflow
  for (value = 0; *hex; hex++) {
    value <<= 4;
    if (*hex >= '0' && *hex <= '9')
      value |= *hex - '0';
    else if (*hex >= 'A' && *hex <= 'F')
      value |= *hex - 'A' + 10;
    else if (*hex >= 'a' && *hex <= 'f')
      value |= *hex - 'a' + 10;
    else
      break;  // stop at first non-hex digit
  }
  return value;
}

//manage data from uart
void manage_data(char *PMS_data) {

  char data[29];
  strcpy(data, PMS_data);

  //Mac
  char address_device;
  unsigned long long int int_mac_device = 0;
  unsigned long long int MAC_device  = 0;
  for (int i = 0; i < 6 ; i++) {
    address_device = data[i];
    //Serial.println(address_device);
    if (('0' <= address_device) && (address_device <= '9')) {
      int_mac_device = ((address_device - '0') * (pow(16, (5 - i))));

    } else if (('A' <= address_device) && (address_device <= 'F')) {
      int_mac_device = ((address_device - 'A' + 10) * (pow(16, (5 - i))));
    }
    else if (('a' <= address_device) && (address_device <= 'f')) {
      int_mac_device = ((address_device - 'a' + 10) * (pow(16, (5 - i))));
    }
    MAC_device += int_mac_device;
  }
  char ID_device[20];
  sprintf(ID_device, "%llu", MAC_device);
  //Serial.println(ID_device);


  //cnt
  uint16_t Cnt_data = 0;
  uint16_t Cnt = 0;
  char address_cnt;
  for (uint8_t i = 0 ; i < 4; i++) {
    address_cnt = data[i + 12];
    if (('0' <= address_cnt)  &&  (address_cnt  <= '9')) {
      Cnt_data =  (address_cnt  - '0')  * (pow(16, (3 - i)));
    } else if (('A' <=  address_cnt)  &&  (address_cnt  <= 'F')) {
      Cnt_data =  (address_cnt  - 'A' + 10) * (pow(16, (3 - i)));
    }
    else if (('a' <=  address_cnt)  &&  (address_cnt  <= 'f')) {
      Cnt_data =  (address_cnt  - 'a' + 10) * (pow(16, (3 - i)));
    }
    Cnt += Cnt_data;
  }
  //Serial.println(Cnt);

  //activity
  char act = data[20];
  uint8_t Act = 0;
  uint8_t Activity = 0 ;
  if (('0' <= act)  &&  (act  <= '9')) {
    Act =  act  - '0';
  } else if (('A' <=  act)  &&  (act  <= 'F')) {
    Act =  act  - 'A' + 10;
  }
  else if (('a' <=  act)  &&  (act  <= 'f')) {
    Act =  act  - 'a' + 10;
  }
  //Serial.println(Act);
  Activity = Act & 15;

  //batt
  uint8_t bitbatt = 7;
  String batt_1;
  batt_1 = data[22] & 7;

  //char batt_2;
  char batt_2 = data[23];
  uint8_t Batt_2 = 0;
  if (('0' <= batt_2)  &&  (batt_2  <= '9')) {
    Batt_2 =  batt_2  - '0';
  } else if (('A' <=  batt_2)  &&  (batt_2  <= 'F')) {
    Batt_2 =  batt_2  - 'A' + 10;
  }
  else if (('a' <=  batt_2)  &&  (batt_2  <= 'f')) {
    Batt_2 =  batt_2  - 'a' + 10;
  }
  uint8_t batt = 0 ;
  batt = (batt_1.toInt() * 16) + int(Batt_2);
  //Serial.println(batt);

  //rssi
  String rssi = String(data[24]) + String(data[25]) + String(data[26]) + String(data[27]);
  //Serial.println(rssi);
  /*
    //variance
    String vars_beacon_data = String(data[6]) + String(data[7]) + String(data[8]) + String(data[9]) + String(data[10]) + String(data[11]);
    float vars = invert_varience(vars_beacon_data) / (pow(10, 6));
    char varience[15];
    dtostrf(vars, 1, 6, varience);
    //Serial.printf("vars_raw_data: %s\tvars: %0d\t", vars_data, varience(vars_data));

    //heading
    String vt_1 = String(data[14]) + String(data[15]) + String(data[16]) + String(data[17]);

    const char * vt_c = vt_1.c_str();
    int16_t vt_int = hex2int(vt_c);
    //vt_int = hex2int(vt_c);
    //Serial.printf("vt_1: %s \tvt_int: %d\t", vt_1, vt_int);

    char address_vt_float;
    unsigned int  vt_2 = 0;
    unsigned int vt_sum  = 0;
    //Serial.print("vt_2: ");
    for (int i = 0; i < 2 ; i++) {
    address_vt_float = data[i + 18];
    //Serial.print(address_vt_float);
    if (('0' <= address_vt_float) && (address_vt_float <= '9')) {
      vt_2 = ((address_vt_float - '0' ) * (pow(16, (1 - i))));

    } else if (('A' <= address_vt_float) && (address_vt_float <= 'F')) {
      vt_2 = ((address_vt_float - 'A' + 10) * (pow(16, (1 - i))));
    }
    else if (('a' <= address_vt_float) && (address_vt_float <= 'f')) {
      vt_2 = ((address_vt_float - 'a' + 10) * (pow(16, (1 - i))));
    }
    vt_sum += vt_2;
    }
    float vt_float = vt_sum / (pow(10, 2));
    //Serial.printf("\tvt_float: %f\r\n", vt_float);

    float vt_heading = 0.0;
    if (vt_int >= 0 )
    vt_heading = vt_int + vt_float;
    else if (vt_int < 0 )
    vt_heading = vt_int - vt_float;
    char angle[15];
    dtostrf(vt_heading, 1, 2, angle);
  */
  String data_tag = "tencent";
  data_tag += ",d=";
  data_tag += ID_device;
  data_tag += ",s=";
  data_tag += ID_station_hardcode;
  data_tag += ",c=";
  data_tag += String(Cnt);
  data_tag += ",t=T";
  data_tag += " ";
  data_tag += "m=0";
  //data_tag += String(Activity);
  data_tag += ",b=";
  data_tag += String(batt);
  data_tag += ",r=";
  data_tag += rssi;
  data_tag += ",a=0";
  //data_tag += String(angle);
  //data_tag += String(vt_heading);
  data_tag += ",v=0";
  //data_tag += String(varience);
  data_tag += ",f=";
  data_tag += String(Activity);

  //Serial.printf("%s   ,%s\r\n", ID_device, rssi);
  //if (MAC_device == 89)Serial.println(data_tag);
  //Serial.print("manage: ");

  //Serial.printf("\rmillis before: %lu \r\n", millis());
  send_mqtt(data_tag);
}


String chk_sum(String boardcast) {

  char sum_chk[20];
  boardcast.toCharArray(sum_chk, sizeof(sum_chk));
  //unsigned long long int int_mac = 0;
  //unsigned long long int MAC  = 0;
  uint8_t int_mac = 0;
  uint8_t MAC = 0;
  char address;
  for (int i = 0; i < boardcast.length(); i++) {
    address = sum_chk[i];
    if (i % 2 == 0) {
      if (('0' <= address) && (address <= '9')) {
        int_mac = ((address - '0' ) * 16);

      } else if (('A' <= address) && (address <= 'F')) {
        int_mac = ((address - 'A' + 10) * 16);
      }
      else if (('a' <= address) && (address <= 'f')) {
        int_mac = ((address - 'a' + 10) * 16);
      }
    } else {
      if (('0' <= address) && (address <= '9')) {
        int_mac = ((address - '0' ));

      } else if (('A' <= address) && (address <= 'F')) {
        int_mac = ((address - 'A' + 10));
      }
      else if (('a' <= address) && (address <= 'f')) {
        int_mac = ((address - 'a' + 10));
      }
    }
    MAC += int_mac;
  }
  String chk_sum = String(MAC & 255, HEX);
  return chk_sum;

}

void reset() {

  EEPROM.begin(EEPROM_SIZE);
  for (int a = 0; a < 130; a++) {
    EEPROM.write(a, 0xFF);
  }
  EEPROM.commit();
  delay(1000);
  EEPROM.end();
  delay(1000);
  for (int b = 0; b < 3; b++) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    delay(500);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    delay(500);
  }
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  hardware_reset = false;
  ets_printf("reboot \n");
  esp_restart();

}
/*
  void rechcek_token() {

  EEPROM.begin(EEPROM_SIZE);
  int check  = EEPROM.read(check_get_addr);
  EEPROM.end();
  if (check != 1) {
    Serial.println("GET_Reftoken");
    Get_RefreshToken();
  }

  EEPROM.begin(EEPROM_SIZE);
  for (int j = 0; j < 200; j++) {
    refresh_token[j] = EEPROM.read(j + token_addr);
  }
  EEPROM.end();
  //Serial.printf("refresh_token %s \n", refresh_token);

  Get_token();

  }
*/
void advertiser_network_gateway() {
  String inst_gateway;
  time_boardcast = millis();
  String str_name = "4150534730" + String(id_station) + "30" ; //10byte
  String sum_dat = chk_sum(str_name);
  inst_gateway = str_name + sum_dat;
  //Serial.println(inst_gateway);
  int name_legth = inst_gateway.length();
  char namespace_c[name_legth + 1];
  strcpy(namespace_c, inst_gateway.c_str());
  NAMESPACE.append(namespace_c);

  INSTANCE  = "000000000000"; //6 byte

  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
  //pAdvertising = pServer->getAdvertising();
  setBeacon(pAdvertising);
  //Serial.println("Start Broadcast Network package");
  pAdvertising->start();
  delay(500);
  //Serial.println("Stop Broadcast Network package");
  pAdvertising->stop();
  NAMESPACE.clear();
  INSTANCE.clear();
}

void setup() {

  Serial.begin(115200);

  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  //pinMode(buzzer, OUTPUT);

  pinMode(button, INPUT_PULLUP);
  attachInterrupt(button, isrfbutton, FALLING);                           //set interrupt button

  BLEDevice::init("Station-0025");
  EEPROMInit();

  StartBeacon();

  EEPROM.begin(EEPROM_SIZE);
  for (int j = 0; j < addr_size_id; j++) {
    if ((EEPROM.read(j + ID_addr) == 0x00) || (EEPROM.read(j + ID_addr) == 0xFF)) {
      //Serial.println("NO EEPROM");
      break;
    } else {
      NODE_ID[j] = EEPROM.read(j + ID_addr);
      id_station += NODE_ID[j];
      //Serial.println(NODE_ID[j]);
      //Serial.printf("read ID_addr %c ID_addr %d \r\n", NODE_ID[j], j + ID_addr);
    }
  }
  EEPROM.end();

  queue_send = xQueueCreate(50, 29);                                      //Create 10 queue, 19 bytes per queue
  if (queue_send == NULL) {
    Serial.println("QUEUE_NULL");
    return;
  }

  xTaskCreate(WiFiStatus, "WiFiStatus", 3000, NULL, 1, NULL);           //Create MultiTask check status WiFi

  esp32.setServer(mqtt_server, mqtt_port);                               // connect mqtt server
  esp32.setCallback(callback);                                           // function callback mqtt
  esp32.connect(mqtt_name, mqtt_user, mqtt_password);
  while (!esp32.connected()) {
    esp32.connect(mqtt_name, mqtt_user, mqtt_password);
    Serial.println("reconnect");
  }
  Serial.println(mqtt_password);

  Serial2.begin(230400, SERIAL_8N1, RXD2, TXD2);                          //Setting Serial UART

  pServer = BLEDevice::createServer();
  pAdvertising = pServer->getAdvertising();

  get_uart.addTask(data_uart);
  data_uart.enable();

}

void loop() {

  //check hardware reset form interrupts button
  if (hardware_reset) {
    reset();
  }

  get_uart.execute();

}

void uart250ms() {
  //check wifi status
  if (wifi_status == 1) {
    Serial2.write('A');

    String check_data;
    nrf_data = "";
    size_data = 0;
    num_uart = 0;
    flag_count = 0;

    //check data from uart
    while (Serial2.available()) {

      char Data = Serial2.read();
      Serial.print(Data);

      if (flag_count == 1) {

        if (Data == ',')  {
          String Data_to_queue = nrf_data;
          //Serial.print("Data_to_queue: ");
          //Serial.println(Data_to_queue);
          char *cstr = new char[Data_to_queue.length() + 1];
          strcpy(cstr, Data_to_queue.c_str());
          xQueueSend(queue_send, &cstr , portMAX_DELAY);

          Data_to_queue = "";
          nrf_data = "";

          //Data_to_queue.clear();
          //nrf_data.clear();
        }
        else if (Data == ':') {
          String Data_to_queue_2 = nrf_data;
          //Serial.print("Data_to_queue_last: ");
          //Serial.println(Data_to_queue_2);
          char *cstr_2 = new char[Data_to_queue_2.length() + 1];
          strcpy(cstr_2, Data_to_queue_2.c_str());
          xQueueSend(queue_send, &cstr_2 , portMAX_DELAY);

          Data_to_queue_2 = "";
          nrf_data = "";

          //Data_to_queue_2.clear();
          //nrf_data.clear();
          break;
        } else {
          nrf_data += Data;
        }
      }

      if ((Data == ',') && (flag_count == 0)) {
        flag_count = 1;
        size_data = check_data.toInt() * 29;
        num_uart = check_data.toInt();
        //Serial.println(num_uart);
      }

      if (size_data == 0) {
        check_data += Data; //data number of scan
      }
    }
    //manage and http post
    int Stack_queue = uxQueueSpacesAvailable(queue_send);
    if (Stack_queue < 50) {
      for (int i = 0; i < (50 - Stack_queue); i++) {
        char *Mac_instance;
        if (xQueueReceive(queue_send, &Mac_instance, portMAX_DELAY) == pdPASS) {
          //Serial.println(Mac_instance);
          manage_data(Mac_instance);
          free(Mac_instance);
        } else {
          Serial.println("can't get queue");
        }
      }
    }
  }
}
