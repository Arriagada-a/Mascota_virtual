#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1   

float temperatura_recibida = 0;
float humedad_recibida = 0;
int luz_recibida =0;
bool luz_encendida = false;  
bool mascota_muerta = false;
bool ventiladorActivo = false;
unsigned long lastPublishTime = 0;
const long publishInterval = 5000; 
unsigned long ventilador= 0;

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define BUZZER_PIN 25
#define pinLDR 34
#define COOLER_PIN 12
#define DHTPIN 4        // Pin donde está conectado el DHT11
#define DHTTYPE DHT11   // Tipo de sensor DHT
DHT dht(DHTPIN, DHTTYPE);



const char* ssid = "SiTSA-WiFi755";
const char* password = "17389428";
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

#include "mascota_reposo.h"
#include "mascota_calor.h"
#include "mascota_muerte.h"
#include "mascota_deprimido.h"
#include "mascota_durmiendo.h"
#include "mascota_acariciar.h"
#include "mascota_comer.h"
#include "mascota_ventilar.h"
#include "mascota_limpiar.h"

const char* topic = "sensor";
const char* topic_subscribe = "sensor/mascota/accion";



float h = 0;
float t = 0;
int valorLDR = 0;


const uint8_t* mascotas1[] = {
    mascota_animada1,
    mascota_animada2,
    mascota_animada3,
    mascota_animada4,
    mascota_animada5
};
const uint8_t* mascotas2[] = {
    mascota_animada6,
    mascota_animada7,
    mascota_animada8,
    mascota_animada9,
    mascota_animada10,
    mascota_animada11,
    mascota_animada12,
    mascota_animada13,
    mascota_animada14
};
const uint8_t* mascotaCalor[] = { 
    mascota_calor1,
    mascota_calor2,
    mascota_calor3,
    mascota_calor4,
    mascota_calor5
};

const uint8_t* mascotaMuerte1[] = {
    mascota_muerte1,
    mascota_muerte2
};

const uint8_t* mascotaMuerte2[] = {
    mascota_muerte3
};

const uint8_t* mascotaDeprimido2[] = {
    mascota_deprimido3,
    mascota_deprimido4,
    mascota_deprimido5
};
const uint8_t* mascotaDurmiendo[] = {
    mascota_durmiendo1,
    mascota_durmiendo2,
    mascota_durmiendo3
};
const uint8_t* mascotaAcariciar[] = {
  mascota_acariciar1,
  mascota_acariciar2,
  mascota_acariciar3,
  mascota_acariciar4
 };

 const uint8_t* mascotaComer[] = {
  mascota_comer1,
  mascota_comer2,
  mascota_comer3,
  mascota_comer4
 };

  const uint8_t* mascotaVentilar[] = {
  mascota_ventilar1,
  mascota_ventilar2,
  mascota_ventilar3
 };
 const uint8_t* mascotaLimpiar[] = {
  mascota_limpiar1,
  mascota_limpiar2,
  mascota_limpiar3
 };

const int num_mascotas1 = sizeof(mascotas1) / sizeof(mascotas1[0]);
const int num_mascotas2 = sizeof(mascotas2) / sizeof(mascotas2[0]);
const int num_mascotaCalor = sizeof(mascotaCalor) / sizeof(mascotaCalor[0]);
const int num_mascotaMuerte1 = sizeof(mascotaMuerte1) / sizeof(mascotaMuerte1[0]);
const int num_mascotaMuerte2 = sizeof(mascotaMuerte2) / sizeof(mascotaMuerte2[0]);
const int num_mascotaDeprimido2 = sizeof(mascotaDeprimido2) / sizeof(mascotaDeprimido2[0]);
const int num_mascotaDurmiendo = sizeof(mascotaDurmiendo) / sizeof(mascotaDurmiendo[0]);
const int num_mascotaAcariciar = sizeof(mascotaAcariciar) / sizeof(mascotaAcariciar[0]);
const int num_mascotaComer = sizeof(mascotaComer) / sizeof(mascotaComer[0]);
const int num_mascotaVentilar = sizeof(mascotaVentilar) / sizeof(mascotaVentilar[0]);
const int num_mascotaLimpiar = sizeof(mascotaLimpiar) / sizeof(mascotaLimpiar[0]);

void setup() {
    Serial.begin(9600);
    
    // Inicializar el sensor DHT11
    dht.begin();
    pinMode(COOLER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(COOLER_PIN, LOW);
    analogSetPinAttenuation(pinLDR, ADC_11db);

    
    // Inicializar la pantalla
    if (!display.begin(0x3C, true)) {
        Serial.println(F("No se pudo iniciar SH1106"));
        for (;;);
    }
    
    display.display();
    delay(2000);
    display.clearDisplay();

    // Conectar a WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando a WiFi...");
    }
    Serial.println("Conectado a WiFi");
    
    // Configurar el cliente MQTT
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqttCallback);

    // Intentar conectar al servidor MQTT solo una vez
    if (client.connect("Agustin")) {
        Serial.println("Conectado al servidor MQTT");
        client.publish("sensor", ""); // Suscribirse a un topic para recibir comandos
        client.subscribe(topic_subscribe);
        client.subscribe(topic);
    } else {
        Serial.print("Error de conexión, rc=");
        Serial.println(client.state());
    }
}


void mostrarMascotas(const uint8_t* mascotas[], int num_mascotas, int repeticiones) {
    for (int i = 0; i < repeticiones; i++) {
        for (int j = 0; j < num_mascotas; j++) {
            display.clearDisplay();
            display.drawBitmap(0, 0, mascotas[j], 128, 64, SH110X_WHITE);
            display.display();  
            delay((j == 0 && i == 0) ? 200 : 200); 
        }
    }
}
void aprobacion() {
  
  for(int i = 0; i < 100; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(1000);
  }
  delay(100);
  
  for(int i = 0; i < 100; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(800);
  }
  delay(100);
  
  for(int i = 0; i < 100; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(600);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(600);
  }
  delay(200);
}

void crecimiento() {
 
  for(int delayTime = 1000; delayTime > 500; delayTime -= 50) {
    for(int i = 0; i < 30; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }
  delay(100);
}

void decrecimiento() {
  
  for(int delayTime = 500; delayTime < 2000; delayTime += 50) { 
    for(int i = 0; i < 50; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }
  delay(100); // Pausa al final
}

void resurrecion() {

  for(int delayTime = 1000; delayTime > 200; delayTime -= 80) { 
    for(int i = 0; i < 15; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }

  
  delay(50);

 
  for(int delayTime = 200; delayTime < 800; delayTime += 50) { 
    for(int i = 0; i < 15; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }
 }

  void cute() {
  // Sonido cute con variación de frecuencias
  for(int delayTime = 1500; delayTime > 1000; delayTime -= 100) { 
    for(int i = 0; i < 10; i++) {  // Ciclo con tono alto
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }
  for(int delayTime = 1000; delayTime < 1500; delayTime += 100) { 
    for(int i = 0; i < 10; i++) {  // Ciclo con tono bajo
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(delayTime);
    }
  }
  delay(100); // Pausa al final
}




void leerSensores() {
   
    float tempTemp = dht.readTemperature();
    float tempHum = dht.readHumidity();
    
   
    if (!isnan(tempTemp) && !isnan(tempHum)) {
        temperatura_recibida = tempTemp;
        humedad_recibida = tempHum;
    }
    
    
    int valorLuz = analogRead(pinLDR);
    luz_recibida = map(valorLuz, 0, 900, 0, 100); 


    
    
    Serial.println("Lecturas de sensores:");
    Serial.print("Temperatura: ");
    Serial.println(temperatura_recibida);
    Serial.print("Humedad: ");
    Serial.println(humedad_recibida);
    Serial.print("Luz: ");
    Serial.println(luz_recibida);
}



void mqtt() {


   
    leerSensores();

   
    StaticJsonDocument<200> doc;
    doc["temperatura"] = temperatura_recibida;
    doc["humedad"] = humedad_recibida;
    doc["luz"] = luz_recibida;
    
   
    char jsonBuffer[200];
    serializeJson(doc, jsonBuffer);
    
 
    client.publish("sensor", jsonBuffer);
    Serial.println("Datos publicados: ");
    Serial.println(jsonBuffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Mensaje recibido en topic: ");
    Serial.println(topic);

    char jsonBuffer[128];
    if (length >= sizeof(jsonBuffer)) {
        Serial.println("Error: Mensaje demasiado largo");
        return;
    }

    memcpy(jsonBuffer, payload, length);
    jsonBuffer[length] = '\0';

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    if (error) {
        Serial.print("Error al deserializar JSON: ");
        Serial.println(error.c_str());
        return;
    }

    if (strcmp(topic, "sensor/mascota/accion") == 0) {
        if (doc.containsKey("accion")) {
            const char* accion = doc["accion"];

            if (strcmp(accion, "morir") == 0) {
                mascota_muerta = true;
                mostrarMascotas(mascotaMuerte1, num_mascotaMuerte1, 3);
                decrecimiento();
                Serial.println("Acción: Mascota muerta");
            } 
            else if (strcmp(accion, "revivir") == 0) {
                if (mascota_muerta) {
                    mascota_muerta = false;
                    resurrecion();
                    Serial.println("¡La mascota ha revivido!");
                }
            } 
            else if (!luz_encendida && !mascota_muerta) {
                if (strcmp(accion, "alimentar") == 0) {
                    mostrarMascotas(mascotaComer, num_mascotaComer, 2);
                    crecimiento();
                    Serial.println("Acción: Alimentando mascota");
                    
                }
                if (strcmp(accion, "carinio") == 0) {
                    mostrarMascotas(mascotaAcariciar, num_mascotaAcariciar, 2);
                    aprobacion();
                    Serial.println("Acción: Acariciando mascota");
                    
                }if (strcmp(accion, "limpiar") == 0) {
                    mostrarMascotas(mascotaLimpiar, num_mascotaLimpiar, 3);
                    cute();
                    Serial.println("Acción: Limpiando mascota");

                }if (strcmp(accion, "jugar") == 0) {
                    mostrarMascotas(mascotas2, num_mascotas2, 1);
                    Serial.println("Acción: Acariciando mascota");
                    
                }if (strcmp(accion, "ventilar") == 0) {
                    ventiladorActivo = true; 
                    digitalWrite(COOLER_PIN, HIGH);
                    ventilador = millis();
                    Serial.println("Cooler encendido");
            } 
                 
            }
        }
    } 
    else if (strcmp(topic, "sensor") == 0) {
        if (doc.containsKey("temperatura") && doc.containsKey("humedad") && doc.containsKey("luz")) {
            temperatura_recibida = doc["temperatura"].as<float>();
            humedad_recibida = doc["humedad"].as<float>();
            luz_recibida = doc["luz"].as<float>();

            Serial.println("Nuevos valores recibidos:");
            Serial.print("Temperatura: ");
            Serial.println(temperatura_recibida);
            Serial.print("Humedad: ");
            Serial.println(humedad_recibida);
            Serial.print("Luz: ");
            Serial.println(luz_recibida);
        }
    }
}

void loop() {
    client.loop();

     
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublishTime >= publishInterval) {
        leerSensores();
        mqtt();
        lastPublishTime = currentMillis;
    }
    
    
    luz_encendida = (luz_recibida <= 70);

   
    if (mascota_muerta) {
        mostrarMascotas(mascotaMuerte2, num_mascotaMuerte2, 1);
        Serial.println("Estado: Mascota muerta");
        delay(1000);
        return;
    }

    
    if (luz_encendida) {
        mostrarMascotas(mascotaDurmiendo, num_mascotaDurmiendo, 3);
        Serial.println("Estado: Luz baja - Mascota durmiendo");
        Serial.print("Valor de luz: ");
        Serial.println(luz_recibida);
        mqtt();
        delay(1000);
        return;
    }
       
    if (ventiladorActivo) {
    mostrarMascotas(mascotaVentilar, num_mascotaVentilar, 2); // Mostrar la animación mientras el ventilador está activo
    Serial.println("Estado: Ventilador activo");

    // Apagar el ventilador automáticamente después de 5 segundos
       if (millis() - ventilador >= 5000) {
          ventiladorActivo = false; // Desactivar el ventilador
          digitalWrite(COOLER_PIN, LOW);
          Serial.println("Cooler apagado automáticamente después de 5 segundos");
     }
     return;
}

    
    if (temperatura_recibida > 30) {
        mostrarMascotas(mascotaCalor, num_mascotaCalor, 1);
        Serial.println("Estado: Temperatura alta");
    } else if (humedad_recibida >= 95) {
        mostrarMascotas(mascotaDeprimido2, num_mascotaDeprimido2, 1);
        Serial.println("Estado: Humedad muy alta");
    } else if (temperatura_recibida <= 30 && humedad_recibida < 95) {
        mostrarMascotas(mascotas1, num_mascotas1, 1);
        Serial.println("Estado: Normal");
    } 

    delay(500);
}