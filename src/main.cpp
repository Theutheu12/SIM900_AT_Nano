#define TINY_GSM_MODEM_SIM900

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <LowPower.h>
#include <avr/wdt.h>

#define onModulePin 9

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "theutheu12"
#define AIO_KEY         "a04a04cc104a414fbf30bf271f7a434d"

// Set serial for debug console (to the SerialMon Monitor, default speed 115200)
#define SerialMon Serial

// MQTT details
const char* broker = "io.adafruit.com";
const int mqttPort = 1883;
const char* mqttUser = "theutheu12";
const char* mqttPassword = "a04a04cc104a414fbf30bf271f7a434d";

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "gprs.swisscom.ch";
const char user[] = "";
const char pass[] = "";

const int count = 90; //90 = 12min

long lastReconnectAttempt = 0;

int i=0;

Adafruit_INA219 ina219;

SoftwareSerial mySerial(7,8);

TinyGsm modem(mySerial);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout);
void power_on();
void power_off();
void mqttCallback(char* topic, byte* payload, unsigned int len);
boolean mqttConnect();
void software_Reboot(void);

void setup() {
  // put your setup code here, to run once:
  pinMode(onModulePin, OUTPUT);
  SerialMon.begin(115200);
  Serial.println("[INIT] Serial monitor initialized");
  mySerial.begin(115200);
  SerialMon.println("[INIT] Serial SIM900 initialized");
  ina219.begin();
<<<<<<< HEAD
  SerialMon.println("[INIT] INA219 initialized");
}

/////////////////////////////////////////////////////////
// loop
////////////////////////////////////////////////////////

void loop()
{

  SerialMon.println("[STEP0] Main program starting");
  SerialMon.println("[STEP1] SIM900 power on");
=======

>>>>>>> parent of 50ee318... Update main.cpp
  power_on();
  delay(100);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("[STEP2] Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("[STEP3] Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

<<<<<<< HEAD
  SerialMon.print("[STEP4] Waiting for network...");
  if (!modem.waitForNetwork())
  {
=======
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
>>>>>>> parent of 50ee318... Update main.cpp
    SerialMon.println(" fail");
    software_Reboot();
  }
  SerialMon.println(" OK");

  SerialMon.print("[STEP5] Connecting to ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
<<<<<<< HEAD
    delay(100);
    SerialMon.println("[STEP15] SIM900 power off");
    power_off();
    delay(10000);
    goto end;
=======
    software_Reboot();
>>>>>>> parent of 50ee318... Update main.cpp
  }
  SerialMon.println(" OK");

  // MQTT Broker setup
  SerialMon.println("[STEP6] MQTT Broker setup");
  mqtt.setServer(broker, mqttPort);
  SerialMon.println("[STEP7] MQTT Callback setup");
  mqtt.setCallback(mqttCallback);

  if (mqtt.connect("SIM900", mqttUser, mqttPassword))
  {
    SerialMon.println("[STEP8] Get Bus voltage");
    float busvoltage = 0;
    busvoltage = ina219.getBusVoltage_V();
    char voltagestring[5];
    char voltage_message[30];
    dtostrf(busvoltage,3,1,voltagestring);
    sprintf(voltage_message, "%s", voltagestring);
    SerialMon.print("[STEP9] Bus Voltage:   ");
    SerialMon.print(busvoltage);
    SerialMon.println(" V");
    SerialMon.println("[STEP10] MQTT publish");
    mqtt.publish("theutheu12/feeds/voltage",voltage_message);

    SerialMon.print("[STEP11] Get Current");
    float current = 0;
    current = ina219.getCurrent_mA();
    char currentstring[5];
    char current_message[30];
    dtostrf(current,3,1,currentstring);
    sprintf(current_message, "%s", currentstring);
    SerialMon.print("[STEP12] Current:   ");
    SerialMon.print(busvoltage);
    SerialMon.println(" mA");
    SerialMon.println("[STEP13] MQTT publish");
    mqtt.publish("theutheu12/feeds/current",current_message);

    mqtt.disconnect();
  }
  else
  {
    SerialMon.println("[STEP14] failed with state ");
    SerialMon.print(mqtt.state());
    delay(2000);
    software_Reboot();
  }


  delay(100);
  SerialMon.println("[STEP15] SIM900 power off");
  power_off();
  delay(100);

  for (size_t i = 0; i < count; i++)
  {
<<<<<<< HEAD
    SerialMon.println("[STEP16] Sleep for 8sec");
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    SerialMon.println("[STEP17] Wake up");
  }

end:
  SerialMon.println("[END] Main program end");
  delay(10);
  SerialMon.println("[END] Restart");
=======
    //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    delay(1000);
  }

  software_Reboot();
}

/////////////////////////////////////////////////////////
// loop
////////////////////////////////////////////////////////

void loop() {


>>>>>>> parent of 50ee318... Update main.cpp
}

// Cette fonction permet d'envoyer des commandes AT au module GSM.
int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    // Initialisation de la chaine de caractère (string).
    memset(response, '\0', 100);

    delay(100);

    // Initialisation du tampon d'entrée (input buffer).
    while( mySerial.available() > 0) mySerial.read();

    // Envoi des commandes AT
    mySerial.println(ATcommand);


    x = 0;
    previous = millis();

    // Cette boucle attend la réponse du module GSM.

    do{
// Cette commande vérifie s'il y a des données disponibles dans le tampon.
//Ces données sont comparées avec la réponse attendue.
        if(mySerial.available() != 0){
            response[x] = mySerial.read();
            x++;
            // Comparaison des données
            if (strstr(response, expected_answer) != NULL)
            {
                answer = 1;
            }
        }
    // Attente d'une réponse.
    }while((answer == 0) && ((millis() - previous) < timeout));

    //SerialMon.println(response); //Cette ligne permet de debuguer le programme en cas de problème !
    return answer;
}

void power_on(){

    uint8_t answer=0;

    // Cette commande vérifie si le module GSM est en marche.
    answer = sendATcommand("AT", "OK", 2000);
    SerialMon.println(answer, DEC);
    if (answer == 0)
    {
        // Mise en marche du module GSM
        digitalWrite(onModulePin,HIGH);
        delay(3000);
        digitalWrite(onModulePin,LOW);

        // Envoie d'une commande AT toutes les deux secondes et attente d'une réponse.
        while(answer == 0){
            answer = sendATcommand("AT", "OK", 2000);
        }
    }

}

void power_off(){
  pinMode(9, OUTPUT);
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);
  delay(3000);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {}

void software_Reboot(void)
{
  SerialMon.print("Reseting");
  wdt_enable(WDTO_15MS);

  while(1)
  {
  }
}
