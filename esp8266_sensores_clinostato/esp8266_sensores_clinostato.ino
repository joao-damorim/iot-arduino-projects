#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <OneWire.h>  
#include <DallasTemperature.h>

#define   pino_D0_1  12 //D6
#define   pino_D0_2  15 //D8
#define   pino_SCL   5  //D1 
#define   pino_SDA   4  //D2
#define   pino_TBI   0  //D3
#define   pino_TUN   2  //D4

Adafruit_MPU6050 mpu;

OneWire oneWireBI(pino_TBI);  //Protocolo OneWire
OneWire oneWireUN(pino_TUN);
/********************************************************************/
DallasTemperature sensorsBI(&oneWireBI); //encaminha referências OneWire para o sensor
DallasTemperature sensorsUN(&oneWireUN); //encaminha referências OneWire para o sensor
/********************************************************************/ 

const char* ssid = ""; //Definir o nome da rede WIFI.
const char* password = ""; //Definir a senha da rede WIFI.
const char* mqtt_server = "";
const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[200];
int value = 0;

int rpm_1;
int rpm_2;
int rpm_1_b;
int rpm_2_b;
unsigned long timeold_1; 
unsigned long timeold_2; 
unsigned long timeold_p;

float ax;
float ay;
float az;

float gx;
float gy;
float gz;

float mpu_temperatura;

void setup() {
  
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");

  sensorsBI.begin(); //inicia biblioteca
  sensorsUN.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  
  pinMode(pino_D0_1, INPUT);
  pinMode(pino_D0_2, INPUT);
//  pinMode(pino_D0_2, INPUT_PULLUP);
  
//  Serial.setDebugOutput(true);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  timeold_1 = millis();
  rpm_1 = 0;
  rpm_1_b = 0;

  timeold_2 = millis();
  rpm_2 = 0;
  rpm_2_b = 0;

  timeold_p = millis();
}

void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world init");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  
  client.loop();

  delay(50);

  if (digitalRead(pino_D0_1) == HIGH){
       
      rpm_1_b = (60000/(millis() - (timeold_1 + 200)));
      timeold_1 = millis();

      if(rpm_1_b > 0){
        rpm_1 = rpm_1_b;
        Serial.print("RPM_1 = ");
        Serial.println(rpm_1, DEC);
      }
  }
      
  if (digitalRead(pino_D0_2) == HIGH){
       
      rpm_2_b = (60000/(millis() - (timeold_2 + 200)));
      timeold_2 = millis();
      //Serial.print("RPM_2 = ");
      //Serial.println(rpm_2, DEC);

      if(rpm_2_b > 0){
        rpm_2 = rpm_2_b;
        Serial.print("RPM_2 = ");
        Serial.println(rpm_2, DEC);
      }
  }
      
  if (millis() - timeold_p >= 30000){

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    ax = a.acceleration.x;
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    ay = a.acceleration.y;
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    az = a.acceleration.z;
    Serial.println(" m/s^2");
  
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    gx = g.gyro.x;
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    gy = g.gyro.y;
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    gz = g.gyro.z;
    Serial.println(" rad/s");
  
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    mpu_temperatura = temp.temperature;
    Serial.println(" degC");

    sensorsBI.requestTemperatures();

    Serial.print("A temperatura é: "); //Printa "A temperatura é:"
    Serial.print(sensorsBI.getTempCByIndex(0)); // Endereço do sensor

    sensorsUN.requestTemperatures();

    Serial.print("A temperatura é: "); //Printa "A temperatura é:"
    Serial.print(sensorsUN.getTempCByIndex(0)); // Endereço do sensor
  
    Serial.println("");
    
    sprintf (msg, "r1 %ld r2 %ld ax %.2f ay %.2f az %.2f gx %.2f gy %.2f gz %.2f mt %.2f tbi %.2f tun %.2f", rpm_1, rpm_2, a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, temp.temperature, sensorsBI.getTempCByIndex(0), sensorsUN.getTempCByIndex(0));
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("clinostato", msg);
    timeold_p = millis();
  }
  //delay(500);
}
