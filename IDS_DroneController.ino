#include <WiFi.h>
#include <WiFiUdp.h>

//LCD screen variables
#include <PCD8544.h>
#define BL 27

//Ultra sonic sensor variables
#define trigPin 2
#define echoPin 21

//Button variables
int inPin = 25;
int val = 0;

//Instantiate specific drone
const char * networkName = "TELLO-FD1C26";
const char * networkPswd = "";
//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "192.168.10.1";
const int udpPort = 8889;
boolean connected = false;

char fromTello[256];

unsigned long timer;


static const byte glyph[] = { B00010000, B00110100, B00110000, B00110100, B00010000 };
static PCD8544 lcd;

uint8_t state = 0;

//Controller movement variables
int pitch = 0;
int roll = 0;
int yaw = 0;
int throttle = 0;
char cmd[256];

//The UDP library
WiFiUDP udp;


void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(inPin, INPUT);

 //LCD screen initialization
  lcd.begin(84, 48);

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(BL, OUTPUT);
  digitalWrite(BL, HIGH);
}

//WiFi connection class
void connectToWiFi(const char * ssid, const char * pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));
  // delete old config
  WiFi.disconnect(true);

  //Initiate connection
  WiFi.begin(ssid, pwd);
  Serial.println("Waiting for WIFI connection...");
}

//Drone connection class
void TelloCommand(char *cmd) {
  //only send data when connected
  if (connected) {
    //Send a packet
    udp.beginPacket(udpAddress, udpPort);
    udp.printf(cmd);
    udp.endPacket();
    Serial.printf("Send [%s] to Tello.\n", cmd);
  }
}

void loop() {
  long duration, distance;
  val = digitalRead(inPin);  // read input value

  //Ultra sonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  //LCD screen line 2
  lcd.setCursor(0, 1);
  lcd.print(distance, DEC);

  //State machine that connects the drone to WiFi and the controller
  switch (state)
  {
    case 0:   //Idle not connected
      //LCD screen line 1
      lcd.setCursor(0, 0);
      lcd.print("Controller");

      if (val == HIGH)
      {
        Serial.println("TEST");
        state = 1;
        connectToWiFi(networkName, networkPswd);
        timer = millis() + 5000;
      }
      break;
    case 1: //Trying to connect

      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.print("Connected to: ");
        Serial.println(networkName);

        udp.begin(WiFi.localIP(), udpPort);
        connected = true;
        TelloCommand("command");
        timer = millis() + 2000;
        state = 2;
      }

      if (millis() > timer)
      {
        state = 0;
      }
      break;

    case 2:   //Connected on ground
      lcd.setCursor(0, 0);
      lcd.print("Connected ");
      
      if (WiFi.status() != WL_CONNECTED)
      {
        WiFi.disconnect(true);
        Serial.println("Disconnected");
        state = 0;
      }
      
      if (distance < 10)
      {
        TelloCommand("takeoff");
        timer = millis() + 1000;
        state = 3;
        Serial.println("takeoff");
      }
      break;

    case 3:   //In air

      lcd.setCursor(0, 0);
      lcd.print("In air       ");

      if (millis() > timer)
      {
        timer = millis() + 20;
        pitch = map(analogRead(34) - 1890, -2000, 2000, -100, 100);
        roll = map(analogRead(35) - 1910, -2000, 2000, -100, 100);
        throttle = map(analogRead(33) - 1910, -2000, 2000, -100, 100);
        yaw = map(analogRead(32) - 1910, -2000, 2000, -100, 100);

        sprintf(cmd, "rc %d %d %d %d", roll, pitch, throttle, yaw);
        TelloCommand(cmd);
        //TelloCommand("battery?");
      }
     /* 
      if (udp.parsePacket() > 0)
      {
        udp.read(fromTello, 256);
        //Serial.print("Tello: ");
        //Serial.println(fromTello);
        lcd.setCursor(0, 2);
        lcd.print("Battery: ");
        lcd.print(fromTello);
        lcd.print("%");
      }*/

      if (val == HIGH) {
        TelloCommand("land");
        timer = millis() + 1000;
        state = 0;
        Serial.println("land");
      }
      break;
  }
  delay(200);
}
