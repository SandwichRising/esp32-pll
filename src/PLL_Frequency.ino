/*
  Project to control ADF4159 for UW Tacoma Research 2024
*/
#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pFreqCharacteristic = NULL;
BLECharacteristic* pControlCharacteristic = NULL;
BLECharacteristic* pHopCharacteristic = NULL;
BLECharacteristic* pRegisterCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool prevDeviceConnected = false;

// Hardware
const int LED_PIN = 2; // hardware esp32 led
const int DATA_PIN = 21; // SDA
const int CLOCK_PIN = 22; // SCL
const int LATCH_PIN = 23;
const int BORN_PIN1 = 5;
const int BORN_PIN2 = 18;
const int BORN_PIN3 = 19;

// Frequency Band Variables
const int BANDS = 8; // total bands
// frequencyBands min/max values are in MHz
const uint32_t frequencyBands[BANDS][2] = {{890, 922},{951, 997},{1039, 1103},{1140,1204},{1196,1285},{1305,1406},{1524,1709},{1815,2105}};
const bool bornSets[BANDS][3] = {{1,1,1},{1,1,0},{0,1,1},{0,1,0},{1,0,1},{1,0,0},{0,0,1},{0,0,0}};

// Register assemblers
uint32_t frequency = 900 * 1000000; // in hz
uint32_t intVal = 36; //set to 900MHz initially
uint32_t fracVal = 0; //set to 900MHz initially
// r0
uint32_t muxVal = 0b0110; // digital lock detect
uint32_t rampOn = 0; // ramp off
// r1
uint32_t phaseAdj = 0;
uint32_t phaseVal = 0;
// r3
uint32_t negBld = 0b101;

// Defaults
uint32_t R0 = 0x30120000;
uint32_t R1 = 0x1;
uint32_t R2 = 0x721000A;
uint32_t R3 = 0x1430083;
uint32_t R41 = 0x180104;
uint32_t R42 = 0x180144;
uint32_t R51 = 0x5;
uint32_t R52 = 0x800005;
uint32_t R61 = 0x6;
uint32_t R62 = 0x800006;
uint32_t R7 = 0x7;

// Helper global variables
bool allRegSendFlag = true; // needs to be in loop instead of interrupt, xmit lag. Run on startup.
bool freqRegSendFlag = false;
bool bleDataOn = true; // false suppresses ble data out, prevents shiftout lag
bool fastPinOn = true; // enables fast pins, requires 24Mhz+ logic analyzing to see output
// timers
unsigned long currentMillis = 0; // reduces calls to millis()
unsigned long broadcastTimer = 1000; // in ms. delay between information broadcasts, must be at least 10's of ms
unsigned long lastBroadcast = 0; // millis() based timer
// xmit to PLL led
bool ledBlinkOn = false;
bool ledState = false;
unsigned long ledOnTime = 0;
unsigned long ledBlinkTimeout = 3; // in ms
// freq hop vars & timers
bool freqHopFlag = false;
bool wasHopping = false; // cleanup flag
uint32_t fhDelay = 1000000; // in us (microseconds)
uint32_t fhStep = 50 * 1000000; // in hz
uint32_t fhSpan = 300 * 1000000; // in hz
uint32_t fhStart = frequency; // updated each freq send
unsigned long lastHop = 0;
unsigned long currentMicros = 0;

// For generating UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID        "50e12000-a21d-4471-b2f0-412147c8399e"
#define FREQUENCY_CHARACTERISTIC_UUID "50e12001-a21d-4471-b2f0-412147c8399e"
#define CONTROL_CHARACTERISTIC_UUID "50e12002-a21d-4471-b2f0-412147c8399e"
#define HOP_CHARACTERISTIC_UUID "50e12003-a21d-4471-b2f0-412147c8399e"
#define REGISTER_CHARACTERISTIC_UUID "50e12004-a21d-4471-b2f0-412147c8399e"
#define LED_CHARACTERISTIC_UUID "50e12010-a21d-4471-b2f0-412147c8399e"

// Funtion Prototypes
void calculateIntFrac(void);
void changeBorn(int);
void updateR0(void);
void updateR1(void);
void updateR3(void);
void shiftOutFast(void);
void latchFast(void);
void sendPLLFreqRegisters(void);
void sendPLLAllRegisters(void);
void packageBLERegisterValues(void);
void frequencyHopHandler(void);
void ledHandler(void);

// BLE Classes/Callbacks
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class FreqCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pFreqCharacteristic) {
    //BLEUUID uuid = pFreqCharacteristic->getUUID();  // uuid debug
    //String uuidString = uuid.toString().c_str();
    //Serial.println(uuidString); // Print the UUID
    String value = pFreqCharacteristic->getValue();
    if (value.length() == 4) {

      // assemble data bytes into frequency request buffer
      uint32_t receivedValue1 = static_cast<int>(value[0]);
      uint32_t receivedValue2 = static_cast<int>(value[1]);
      uint32_t receivedValue3 = static_cast<int>(value[2]);
      uint32_t receivedValue4 = static_cast<int>(value[3]);
      uint32_t buffer = receivedValue1 + (receivedValue2 << 8) + (receivedValue3 << 16) + (receivedValue4 << 24);

      for (int i = 0; i < BANDS; i++) { // check input against valid frequencies
        if (buffer >= (frequencyBands[i][0] * 1000000) && buffer <= (frequencyBands[i][1] * 1000000)) {

          //Serial.print("Characteristic event, written (freq): ");
          //Serial.println(buffer); // Print the accepted frequency

          frequency = buffer;
          fhStart = frequency; // freq hop update
          calculateIntFrac();
          updateR0();
          updateR1();
          updateR3();
          changeBorn(i);
          //sendPLLFreqRegisters();
          freqRegSendFlag = true;
          i = BANDS; //exit loop

          // serial debug
          /*Serial.print("intVal: ");
          Serial.println(intVal);
          Serial.print("fracVal: ");
          Serial.println(fracVal);
          Serial.print("R0: 0x");
          Serial.println(R0,HEX);
          Serial.print("R1: 0x");
          Serial.println(R1,HEX);
          Serial.print("R3: 0x");
          Serial.println(R3,HEX); */
        }
      }
    }
  }
};

class ControlCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pControlCharacteristic) {
    String value = pControlCharacteristic->getValue();
    if (value.length() > 0) {
      //Serial.print("Characteristic control event, written: ");
      //Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {  // send all registers
        //sendPLLAllRegisters();
        allRegSendFlag = true;

      } else if (receivedValue == 2) { // send freq registers
        //sendPLLFreqRegisters();
        freqRegSendFlag = true;

      } else if (receivedValue == 3) { // restore defaults
        // freqs
        frequency = 900000000;
        calculateIntFrac();
        changeBorn(0);
        updateR0();
        updateR1();
        updateR3();
        //sendPLLAllRegisters();
        allRegSendFlag = true;

        // fhop defaults
        freqHopFlag = false;
        fhDelay = 1000000;
        fhStep = 50 * 1000000;
        fhSpan = 300 * 1000000;
        fhStart = frequency; 
        // other settings
        bleDataOn = true;
        fastPinOn = true;

      } else if (receivedValue == 4) { // reset esp32
        ESP.restart();

      } else if (receivedValue == 5) { // ble data suppression
        bleDataOn = true; 

      } else if (receivedValue == 6) { 
        bleDataOn = false; 

      } else if (receivedValue == 7) { // fast/slow pin modes
        fastPinOn = true;

      } else if (receivedValue == 8 && fhDelay >= 250) {
        fastPinOn = false;
      }
    }
  }
};

class HopCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pHopCharacteristic) {

    //Serial.print("Hop callback triggered");

    String value = pHopCharacteristic->getValue();
    if (value.length() > 0) {

      // assemble data bytes into index and buffer
      uint32_t receivedIndex = static_cast<int>(value[0]);
      uint32_t receivedValue1 = static_cast<int>(value[1]);
      uint32_t receivedValue2 = static_cast<int>(value[2]);
      uint32_t receivedValue3 = static_cast<int>(value[3]);
      uint32_t receivedValue4 = static_cast<int>(value[4]);
      uint32_t buffer = receivedValue1 + (receivedValue2 << 8) + (receivedValue3 << 16) + (receivedValue4 << 24);
      
      /*
      Serial.print("Hop index/buffer received: ");
      Serial.println(receivedIndex); // Print the integer value
      Serial.println(buffer); // Print the integer value
      */

      // Delay
      if (receivedIndex == 0) {
        if (buffer < 250 && buffer >= 30 && fastPinOn) {
          fhDelay = buffer;
        }
        if (buffer >= 250 && buffer <= 10000000) { // 250us to 10 secs
          fhDelay = buffer;
        }
      // Step  
      } else if (receivedIndex == 1) {
        if (buffer >= 1 && buffer <= (1000 * 1000000) && buffer < fhSpan) { // 1Hz to 1GHz and smaller than span
          fhStep = buffer;
        }
      // Span  
      } else if (receivedIndex == 2) {
        if (buffer >= 10 && buffer <= (2000 * 1000000) && buffer > fhStep) { // 10Hz to 2GHz and bigger than step
          fhSpan = buffer;
        }  
      // Freq Hop On
      } else if (receivedIndex == 3) {
        freqHopFlag = true;
      // Freq Hop Off
      } else if (receivedIndex == 4) {
        freqHopFlag = false;
      }
    }
  }
};

class LedCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    String value = pLedCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
};


// SETUP
void setup() {
  Serial.begin(115200);

  // Setup data pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(BORN_PIN1, OUTPUT);
  pinMode(BORN_PIN2, OUTPUT);
  pinMode(BORN_PIN3, OUTPUT);

  digitalWrite(BORN_PIN1, HIGH);
  digitalWrite(BORN_PIN2, HIGH);
  digitalWrite(BORN_PIN3, HIGH);

  // Create the BLE Device
  BLEDevice::init("PLL-ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a frequency change Characteristic
  pFreqCharacteristic = pService->createCharacteristic(
                      FREQUENCY_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the control buttons Characteristic
  pControlCharacteristic = pService->createCharacteristic(
                      CONTROL_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Create the freq hop Characteristic
  pHopCharacteristic = pService->createCharacteristic(
                      HOP_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create the register send-to-BLE Characteristic
  pRegisterCharacteristic = pService->createCharacteristic(
                      REGISTER_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callbacks for all characteristics
  pFreqCharacteristic->setCallbacks(new FreqCharacteristicCallbacks());
  pControlCharacteristic->setCallbacks(new ControlCharacteristicCallbacks());
  pHopCharacteristic->setCallbacks(new HopCharacteristicCallbacks());
  pLedCharacteristic->setCallbacks(new LedCharacteristicCallbacks());

  // Create BLE Descriptors
  pControlCharacteristic->addDescriptor(new BLE2902());
  pFreqCharacteristic->addDescriptor(new BLE2902());
  pRegisterCharacteristic->addDescriptor(new BLE2902());
  pHopCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

    // Set MTU size - fix attempt for android browser
  //BLEDevice::setMTU(517);
  BLEDevice::setMTU(127);

  // Set preferred connection parameters - fix attempt for android browser
 // BLEDevice::setConnectionParams(0x0006, 0x000c, 0, 100);

  // Set preferred PHY options (1 Mbps, 2 Mbps, or coded) - fix attempt for android browser
  //BLEDevice::setPreferredPhy(ESP_BLE_2M_PHY, ESP_BLE_2M_PHY, ESP_BLE_PHY_CODED);


  // Init PLL Registers
  sendPLLAllRegisters();
}

// LOOP
void loop() {

  currentMillis = millis();
  currentMicros = micros();

  frequencyHopHandler();

  // Send PLL Registers flag resoultion
  if (allRegSendFlag == true) {
    sendPLLAllRegisters();
    allRegSendFlag = false;
  }
  if (freqRegSendFlag == true) {
    sendPLLFreqRegisters();
    freqRegSendFlag = false;
  }

  // notify frequency over bluetooth (heartbeat)
  if (deviceConnected) {
    if (currentMillis >= (lastBroadcast + broadcastTimer) && bleDataOn){
      pFreqCharacteristic->setValue(String(frequency).c_str());
      pFreqCharacteristic->notify();
      packageBLERegisterValues();
      packageBLEFreqhopValues();
      lastBroadcast = currentMillis;
    }
  }

  // disconnecting
  if (!deviceConnected && prevDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    prevDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !prevDeviceConnected) {
    // do stuff here on connecting
    prevDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }

  ledHandler();

}


// FUNCTIONS

void calculateIntFrac(){
  intVal = frequency / 25000000;
  fracVal = int(round(float((frequency - (intVal * 25000000)) * 1.34217727)));
}

void changeBorn(int bandInput) {

  if (fastPinOn) {
    if (bornSets[bandInput][0]) {
      GPIO.out_w1ts = (1 << BORN_PIN1); // Set high
    } else {
      GPIO.out_w1tc = (1 << BORN_PIN1); // Set low
    }

    if (bornSets[bandInput][1]) {
      GPIO.out_w1ts = (1 << BORN_PIN2); // Set high
    } else {
      GPIO.out_w1tc = (1 << BORN_PIN2); // Set low
    }

    if (bornSets[bandInput][2]) {
      GPIO.out_w1ts = (1 << BORN_PIN3); // Set high
    } else {
      GPIO.out_w1tc = (1 << BORN_PIN3); // Set low
    }
  } else { // slow mode
    digitalWrite(BORN_PIN1, bornSets[bandInput][0]);
    digitalWrite(BORN_PIN2, bornSets[bandInput][1]);
    digitalWrite(BORN_PIN3, bornSets[bandInput][2]);
  }

}

void updateR0(){
  R0 = (rampOn << 31) + (muxVal << 27) + (intVal << 15) + ((fracVal >> 13) << 3);
}
void updateR1(){
  R1 = (phaseAdj << 28) + ((fracVal << 19) >> 4) + (phaseVal << 3) + 0b1;
}
void updateR3(){
  if (frequency <= (1370 * 1000000)) {
    negBld = 0b101;
  } else {
    negBld = 0b100;
  }
  R3 = 0x30083 + (negBld << 22);
}

void shiftOutFast(uint8_t val) { // MSB first

  for (int i = 7; i >= 0; i--) {
    // Write bits to the data pin
    if (val & (1 << i)) {
      GPIO.out_w1ts = (1 << DATA_PIN);
    } else {
      GPIO.out_w1tc = (1 << DATA_PIN);
    }
    // Clock high
    GPIO.out_w1ts = (1 << CLOCK_PIN);
    // Clock low
    GPIO.out_w1tc = (1 << CLOCK_PIN);
  }

}

void latchFast(){

  // Latch high
  GPIO.out_w1ts = (1 << LATCH_PIN);
  // Latch low
  GPIO.out_w1tc = (1 << LATCH_PIN);

}

void sendPLLFreqRegisters(){

    uint32_t sendRegisters[3] = {R3, R1, R0};
    for (int i = 0; i < 3; i++){ 
      for (int j = 3; j >= 0; j--) { // increment from MS byte to LS byte
        if (fastPinOn) shiftOutFast((sendRegisters[i] >> (j * 8)));
        else shiftOut(DATA_PIN, CLOCK_PIN, true, (sendRegisters[i] >> (j * 8))); // true = MSB first
      }
        if (fastPinOn) latchFast();
        else {
          digitalWrite(LATCH_PIN, HIGH);
          digitalWrite(LATCH_PIN, LOW);
       }
    }

    ledBlinkOn = true;
}

void sendPLLAllRegisters(){

    uint32_t sendRegisters[11] = {R7, R62, R61, R52, R51, R42, R41, R3, R2, R1, R0};
    for (int i = 0; i < 11; i++){ 
      for (int j = 3; j >= 0; j--) { // increment from MS byte to LS byte
        if (fastPinOn) shiftOutFast((sendRegisters[i] >> (j * 8)));
        else shiftOut(DATA_PIN, CLOCK_PIN, true, (sendRegisters[i] >> (j * 8))); // true = MSB first
      }
        if (fastPinOn) latchFast();
        else {
          digitalWrite(LATCH_PIN, HIGH);
          digitalWrite(LATCH_PIN, LOW);
       }
    }

    ledBlinkOn = true;
}

void packageBLERegisterValues(){
  const int DATAQTY = 13;
  uint32_t data[DATAQTY] = { intVal, fracVal, R0, R1, R2, R3, R41, R42, R51, R52, R61, R62, R7 };
  uint8_t dataArray[DATAQTY*4] = {0};
  for (int i = 0; i < DATAQTY; i++) {
    dataArray[i * 4] = (data[i] & 0xFF);
    dataArray[i * 4 + 1] = (data[i] >> 8) & 0xFF;
    dataArray[i * 4 + 2] = (data[i] >> 16) & 0xFF;
    dataArray[i * 4 + 3] = (data[i] >> 24) & 0xFF;
  }

  pRegisterCharacteristic->setValue(dataArray, DATAQTY*4);
  pRegisterCharacteristic->notify();
}

void packageBLEFreqhopValues(){
  const int DATAQTY = 3;
  uint32_t data[DATAQTY] = { fhDelay, fhStep, fhSpan };
  uint8_t dataArray[DATAQTY*4] = {0};
  for (int i = 0; i < DATAQTY; i++) {
    dataArray[i * 4] = (data[i] & 0xFF);
    dataArray[i * 4 + 1] = (data[i] >> 8) & 0xFF;
    dataArray[i * 4 + 2] = (data[i] >> 16) & 0xFF;
    dataArray[i * 4 + 3] = (data[i] >> 24) & 0xFF;
  }

  pHopCharacteristic->setValue(dataArray, DATAQTY*4);
  pHopCharacteristic->notify();
}


void frequencyHopHandler() {

  if (freqHopFlag) {

    if (!wasHopping) {
      if (fhDelay <= 50000) broadcastTimer = 50;
      else if (fhDelay < 1000000) broadcastTimer = (fhDelay / 1000);
      //Serial.println(broadcastTimer);
      wasHopping = true;
    }

    if (currentMicros > lastHop + fhDelay) {
      //hop!

      uint32_t buffer = frequency;
      bool hopComplete = false;

      while (!hopComplete){

        buffer += fhStep;
        if (buffer > fhStart + fhSpan) buffer = fhStart;

        for (int i = 0; i < BANDS; i++) { // check input against valid frequencies
          if (buffer >= (frequencyBands[i][0] * 1000000) && buffer <= (frequencyBands[i][1] * 1000000)) {

            frequency = buffer;
            calculateIntFrac();
            updateR0();
            updateR1();
            updateR3();
            changeBorn(i);
            //sendPLLFreqRegisters();
            freqRegSendFlag = true;
            hopComplete = true; //exit loop
            i = BANDS; //exit loop
          }
        }
      }

      lastHop = currentMicros;
    }

  } else if (wasHopping) {
    broadcastTimer = 1000;
    wasHopping = false;
  }
}

void ledHandler() {
  if (ledBlinkOn) {
    if (!ledState) {
      // on
      ledOnTime = currentMillis;
      ledState = true;
      GPIO.out_w1ts = (1 << LED_PIN);
    }
    else if (currentMillis > ledOnTime + ledBlinkTimeout) {
      // off
      GPIO.out_w1tc = (1 << LED_PIN);
      ledState = false;
      ledBlinkOn = false;
    }
  }
}