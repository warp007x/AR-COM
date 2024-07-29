#include <Arduino.h>
#include <ezButton.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define VIB      9 // ESP32 pin GPIO9 Vibration Motor
#define VRX_PIN  8  // ESP32 pin GPIO2 VRX pin
#define VRY_PIN  7  // ESP32 pin GPIO3 VRY pin
#define SW_PIN   6  // ESP32 pin GPIO1 SW  pin

#define JOY_ENB  2
#define BTN_ENB  1

#define BT2      33
#define BT3      34
#define BT4      37
#define BT5      38

#define MYPORT_TX 15
#define MYPORT_RX 16

EspSoftwareSerial::UART myPort;
#define HW_SERIAL Serial
#define SW_SERIAL myPort

long lastMsg = 0;
char msg[50];

int valueX = 0; // to store the X-axis value
int valueY = 0; // to store the Y-axis value
int bValueM = 0; // To store value of the button
int bValueU = 0; // To store value of the button
int bValueD = 0; // To store value of the button
int bValueL = 0; // To store value of the button
int bValueR = 0; // To store value of the button

ezButton buttonM(SW_PIN);

void BTLoop(){
  buttonM.loop();
}

void setup() {
  HW_SERIAL.begin(115200); // Standard hardware serial port
  pinMode(JOY_ENB, OUTPUT);
  digitalWrite(JOY_ENB, HIGH);
  pinMode(BTN_ENB, OUTPUT);
  digitalWrite(BTN_ENB, HIGH);
  delay(1000);
  SW_SERIAL.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!SW_SERIAL) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  } 
  // put your setup code here, to run once:
  pinMode(VIB, OUTPUT);
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(SW_PIN, INPUT);
  pinMode(BT2, INPUT);
  pinMode(BT3, INPUT);
  pinMode(BT4, INPUT);
  pinMode(BT5, INPUT);
  buttonM.setDebounceTime(40); // set debounce time to 50 milliseconds
  digitalWrite(VIB, HIGH);
  delay(2000);
  digitalWrite(VIB, LOW);
  HW_SERIAL.println("HELLO");


}

void loop() {
  // put your main code here, to run repeatedly:
  BTLoop();
  valueX = analogRead(VRX_PIN);
  valueY = analogRead(VRY_PIN);
  bValueM = buttonM.getState();
  // bValueU = map(analogRead(BT2), 400, 4095, 0, 1);
  // bValueL = digitalRead(BT3);
  // bValueD = map(analogRead(BT4), 400, 4095, 0, 1);
  // bValueR = digitalRead(BT5);
  bValueU = digitalRead(BT2);
  bValueL = digitalRead(BT3);
  bValueD = digitalRead(BT4);
  bValueR = digitalRead(BT5);

  int valueX_M = map(valueX, 0, 4095, 0, 255);
  int valueY_M = map(valueY, 0, 4095, 0, 255);
  
  if (buttonM.isPressed()) {
    HW_SERIAL.println("The button is pressed");
    // TODO do something here
    digitalWrite(VIB, HIGH);
    delay(200);
    digitalWrite(VIB, LOW);
  }

  if (buttonM.isReleased()) {
    HW_SERIAL.println("The button is released");
    digitalWrite(VIB, LOW);
    // TODO do something here
  }
  // print data to Serial Monitor on Arduino IDE
  // Serial.print("button-M = ");
  // Serial.print(bValueM);
  // Serial.print(", x = ");
  // Serial.print(valueX_M);
  // Serial.print(", y = ");
  // Serial.println(valueX_M);
  // delay(1000);
  StaticJsonDocument<500> JSONbuffer;
//  initialize the object    
    JsonObject JSONencoder = JSONbuffer.createNestedObject();
    long now = millis();
    if (now - lastMsg > 100) {
      lastMsg = now;
          //  define the object parameters
          JSONencoder["jx"]  = valueX_M;
          JSONencoder["jy"]  = valueY_M;
          JSONencoder["BT1"] = bValueM;
          JSONencoder["BT2"] = bValueU;
          JSONencoder["BT3"] = bValueD;
          JSONencoder["BT4"] = bValueL;
          JSONencoder["BT5"] = bValueR;
          char JSONmessageBuffer[500];
          serializeJson(JSONencoder, JSONmessageBuffer);
          HW_SERIAL.println(JSONmessageBuffer);
          SW_SERIAL.println(JSONmessageBuffer);

        // if (myPort.available())
        //   HW_SERIAL.write(SW_SERIAL.read());
    }
    // delay(200);
}

