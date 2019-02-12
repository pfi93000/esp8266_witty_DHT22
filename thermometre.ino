// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// Depends on the following Arduino libraries:
// - Adafruit Unified Sensor Library: https://github.com/adafruit/Adafruit_Sensor
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library

// Modifié par Paul FIEVET - février 2019

// schema électrique pour un esp8266 witty
// 
// GPIO 14 vers patte 2 du DHT 22 (données)
// GND  vers patte 4 du DHT 22
// resistance de 10 kOhms (marron noir orange) entre patte 1 et patte 2 du DHT 22
// GPIO 5 patte 1 du DHT 22 (alimentation en 3.3Volts)
// patte 3 du DHT22 non utilisée
//
// GPIO 16 relié à la patte REST (reset)
//
// GND, VCC, RX, TX, REST et GPIO 0  vers le support de programmation


// pinout d'un ESP8266 witty 
//
// LDR (capteur de lumière)   bouton    LED RGB (3 LED rouge/verte/bleue)
//                                  Led bleue interne - builin
// RESET                                TX
// A0 (sur le capteur de lumière)       RX
// CH_PD (Chip Power-Down)              D1 (GPIO5)
// D0 (GPIO16)                          D2 (GPIO4 - sur le bouton du haut)
// D5 (GPIO14)                          D3 (GPIO0 - sur le Flash Button)
// D6 (GPIO12 - sur la LED Verte)       D4 (GPIO2 - sur la LED bleue interne - builtin)
// D7 (GPIO13 - sur la LED Bleue)       D8 (GPIO15 - sur la LED rouge)
// VCC (5 Volts)                        GND (Masse)


#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            14 // GPIO 14         // Pin which is connected to the DHT sensor.

// personnellement, j'utilise un DHT 22. J'ai donc activé cette ligne
// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;



const int rouge = 15;      // broche qui pilote la LED rouge
const int verte = 12;      // broche qui pilote la LED verte
const int bleue = 13;      // broche qui pilote la LED bleue
const byte bouton = D2;    // broche qui écoute l'état du bouton poussoir
const int lumiere = A0;    // broche qui écoute le capteur de lumière (noté LDR en anglais soit les initiales de Light Dependent Resistor)
const int ledinterne = 2 ; //broche qui pilote la LED interne

const byte interruptPin = bouton; //its pulled high, so when you press it it returns 0, releasing returns 1
volatile byte debug = LOW;   //variable volatile car utilisée dans une interruption 


#include <ESP8266WiFi.h>
#include "ThingSpeak.h"

// Wi-Fi Settings
const char* ssid = "votre_nom_de_reseau_wifi"; // your wireless network name (SSID)
const char* password = "votre_mot_de_passe_reseau_wifi"; // your Wi-Fi network password

WiFiClient client;

// un compte gratuit (ou payant) sur le site de ThingSpeak est obligatoire.
// une fois le compte créé, créer votre "channel"
// récupérez ensuite votre identifiant de channel (channelID), et vos cles API (en écriture et en lecture)
// ThingSpeak Settings
const int channelID = 123456;   // votre numéro de channel sur le site de ThingSpeak
const char * writeAPIKey = "XXXXX"; // write API key for your ThingSpeak Channel
const char * ReadAPIKey =  "YYYYY";
const char* server = "api.thingspeak.com";
const int postingInterval = 600; // post data every 16 seconds


// cette fonction est appellee par l'interruption matétielle liée au bouton
void active() {
    detachInterrupt(digitalPinToInterrupt(interruptPin));  // on desactive l'interruption car le debug est maintenant activé pour toujours
    debug=HIGH;                                            // mise à jour de la variable 'volatile' debug
    Serial.println("Bouton appuye");                       // affiche un message
}


void setup() {
  Serial.begin(115200);
  Serial.println("debut");
  //a chaque redémarrage, le debug est arrêté
  debug=LOW;
  
  //activation de l'interruption materielle sur le bouton pour declancher la fonction "active()"
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), active, LOW);         //bouton appuyé
  attachInterrupt(digitalPinToInterrupt(interruptPin), active, FALLING);     //bouton appuyé (en cours)
  attachInterrupt(digitalPinToInterrupt(interruptPin), active, RISING);      //bouton relaché (en cours)
  attachInterrupt(digitalPinToInterrupt(interruptPin), active, CHANGE);      //changement d'état du bouton

  //Pas d'alimentation en 3.3V du DHT 22 au début
  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW);

  //definition des LED
  pinMode(rouge, OUTPUT);
  pinMode(verte, OUTPUT);
  pinMode(bleue, OUTPUT);
  digitalWrite(rouge, LOW);
  digitalWrite(verte, LOW);
  digitalWrite(bleue, LOW);

  if (debug == HIGH) Serial.println("debug en cours");

  if (debug == HIGH) Serial.println("Connexion WIFI ...");
  //wifi obligatoirement avant la températeur, sinon bug dans le wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(250);
      digitalWrite(rouge, HIGH);
      delay(250);
      digitalWrite(rouge, LOW);
  }
  if (debug == HIGH) Serial.println("Connexion WIFI OK");

  // Initialize DHT device 
  dht.begin();
  if (debug == HIGH) Serial.println("DHTxx Unified Sensor Example");
  // temperature sensor 
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  if (debug == HIGH) {
    Serial.println("------------------------------------");
    Serial.println("Temperature");
    Serial.print  ("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print  ("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" *C");
    Serial.print  ("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" *C");
    Serial.print  ("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" *C");  
    Serial.println("------------------------------------");
  }

  // humidity sensor
  dht.humidity().getSensor(&sensor);
  if (debug == HIGH) {
    Serial.println("------------------------------------");
    Serial.println("Humidity");
    Serial.print  ("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print  ("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println("%");
    Serial.print  ("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println("%");
    Serial.print  ("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println("%");  
    Serial.println("------------------------------------");
  }
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // Get temperature event and print its value.
  //alimentation 3.3V du DHT 22 pendant le temps de la lecture
  digitalWrite(D1, HIGH);
  // pause minimale entre 2 lectures (imposé par la technique du DHT 22)
  delay(delayMS);
  //et une seconde
  delay(1000); 
  sensors_event_t temp; //reserve un espace memoire pour les donnees de temperature
  dht.temperature().getEvent(&temp);   //lecture et remplissage de la zone temp
  sensors_event_t humi; //reserve un espace memoire pour les donnees de l'humidite
  dht.humidity().getEvent(&humi);      //variable humi = lecture de l'humidite
  //arrêt de l'alimentation en 3.3V du DHT 22
  digitalWrite(D1, LOW);

  

  //En hiver, on hiberne la nuit (la batterie se décharge plus vite qu'elle ne se charge) sans rien envoyer à ThingSpeak
  if (!(isnan(temp.temperature))) {
      if(temp.temperature < 15) { // moins de 15°C
          if(analogRead(A0) < 10) {  // peu de lumière
              Serial.print("Temperature: ");
              Serial.print(temp.temperature);
              Serial.println(" *C");
              if (debug == LOW) {
                  Serial.println("Pause hivernale");
                  digitalWrite(rouge, LOW);
                  digitalWrite(verte, LOW);
                  digitalWrite(bleue, LOW);
                  ESP.deepSleep(30 * 60 * 1000000);
                  //après cette ligne plus rien ne s'exécute. Car le processeur s'est arrêté et va rebooter complètement après 30 minutes
              } else {
                  Serial.println("Normalement, en pause hivernale. En debug, on continue");
              }
          }
      }
  }




  //initialisation de flux vers ThingSpeak
  ThingSpeak.begin(client);
  Serial.println("Client ThingSpeak");

  //la valeur lue sur le capteur de lumière est remontée sur le site web ThingSpeak
  ThingSpeak.writeField(channelID, 1, analogRead(A0), writeAPIKey);
  Serial.println("remontee de la lumière faite");
  
  //gestion des données reçues par le capteur DHT22
  if (!(isnan(temp.temperature))) {
    //la température est remontée sur le site web ThingSpeak
    delay(16000); // delais imposé par ThingSpeak pour un compte gratuit
    digitalWrite(bleue, HIGH);
    delay(500);
    digitalWrite(bleue, LOW);
    ThingSpeak.writeField(channelID, 2, temp.temperature, writeAPIKey);
    Serial.println("remontee de la temperature faite");
    if (debug == HIGH) {
        Serial.print("Temperature: ");
        Serial.print(temp.temperature);
        Serial.println(" *C");
    }
  } else {
        if (debug == HIGH) {
            Serial.println("Error reading temperature!");
        }
  }
  // Get humidity event and print its value.
  if (!(isnan(temp.relative_humidity))) {
     //l'humidité est remontée, après le delais imposé par ThingSpeak
    delay(16000);
    digitalWrite(verte, HIGH);
    delay(500);
    digitalWrite(verte, LOW);
    ThingSpeak.writeField(channelID, 3, humi.relative_humidity, writeAPIKey);
    Serial.println("remontee de l'humidite faite");
    if (debug == HIGH) {
        Serial.print("Humidity: ");
        Serial.print(humi.relative_humidity);
        Serial.println("%");
    }
  } else {
        if (debug == HIGH) {
            Serial.println("Error reading humidity!");
        }
  }

  digitalWrite(rouge, LOW);
  digitalWrite(verte, LOW);
  digitalWrite(bleue, LOW);
  Serial.println("calcul du sommeil");

  if (debug == LOW) {
  //sommeil très profond. Un reset complet de l'ESP sera provoqué à la fin du temps, via la patte GPIO 16 qui doit obligatoirement être connectée à la patte RESET
  //la capture de lumière stature à 1024 (son maximum) en plein jour.
  if(analogRead(A0) > 1000) {
    //en pleine journée...
    if(temp.temperature > 25.0) {
        //il fait jour et très chaud (on va attendre le frais)
        Serial.println("jour et plus de 25 degres : 30 min de pause");
        ESP.deepSleep(30 * 60 * 1000000);
    } else {
        if(temp.temperature > 18.0) {
          //il fait jour est frais
          Serial.println("jour et entre 18 degres et 25 degres : 15 minutes de pause");
          ESP.deepSleep(15 * 60 * 1000000);
        } else {
          //il fait jour et froid
          Serial.println("jour et moins de 18 degres : 45 minutes de pause");
          ESP.deepSleep(45 * 60 * 1000000);
        }
    }
  } else {
    //maintenant que la nuit tombe, fait-il plus frais ?
    if(temp.temperature > 24.0) {
        //il fait nuit et très chaud (on va attendre le frais)
        Serial.println("nuit et plus de 24 degres : 15 minutes");
        ESP.deepSleep(15 * 60 * 1000000);
    } else {
        if(temp.temperature > 18.0) {
          //il fait nuit est frais
          Serial.println("nuit et entre 18 degres et 24 degres : 30 minutes");
          ESP.deepSleep(30 * 60 * 1000000);
        } else {
          //il fait nuit et froid
          Serial.println("nuit et moins de 18 degres : 60 minutes");
          ESP.deepSleep(60 * 60 * 1000000);
        }
    }
  }
  } else {
      Serial.println("Pas de pause en mode DEBUG");
      Serial.print("temperature : ");
      Serial.println(temp.temperature);
      Serial.print("Lumière : ");
      Serial.println(analogRead(A0));
      //on arrive ici qu'en mode DEBUG (sans hibernation)
      Serial.println("Arret dans 14 secondes, puis redemarrage au bout de 1 seconde...");
      delay(14000);  //14s
      ESP.deepSleep(1000000); //+1s
  }
  //en dessous de cette ligne plus rien ne s'exécute, car l'ESP est en sommeil profond, et seul un reset peut le réveiller.

}

void loop() {
    //on n'arrive jamais ici
    Serial.println("boucle...");
    ESP.deepSleep(1000000); //+1s
}
