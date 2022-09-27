
/*
    Control meArm using V7RC app
*/

#include "BLEDevice.h"

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLEAdvertData advdata;
BLEAdvertData scndata;

#include <AmebaServo.h>

#define FRONT_SERVO_PIN PA13
#define FRONT_SERVO_MIN 1200
#define FRONT_SERVO_MAX 1800
#define FRONT_SERVO_CENTER 1500
#define FRONT_SERVO_DEADZONE 50

#define REAR_SERVO_PIN PA12
#define REAR_SERVO_MIN 1200
#define REAR_SERVO_MAX 1800
#define REAR_SERVO_CENTER 1500
#define REAR_SERVO_DEADZONE 50

#define ROTATE_SERVO_PIN PA25
#define ROTATE_SERVO_MIN 1200
#define ROTATE_SERVO_MAX 1800
#define ROTATE_SERVO_CENTER 1500
#define ROTATE_SERVO_DEADZONE 50

#define CLAMP_SERVO_PIN PA26
#define CLAMP_SERVO_MIN 1200
#define CLAMP_SERVO_MAX 1800
#define CLAMP_SERVO_CENTER 1500
#define CLAMP_SERVO_DEADZONE 50

AmebaServo frontArmServo, rearArmServo, rotateServo, clampServo;

bool updated = false;
String rxString = "";

void writeCB (BLECharacteristic* chr, uint8_t connID) {
    if (updated == false && chr->getDataLen() > 0) {
        rxString = chr->readString();
        updated = true;
    }
}

void setup() {
    Serial.begin(115200);

    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMEBA_BLE_DEV");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteProperty(true);
    Rx.setWriteNRProperty(true);
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();

    frontArmServo.attach(FRONT_SERVO_PIN);
    rearArmServo.attach(REAR_SERVO_PIN);
    rotateServo.attach(ROTATE_SERVO_PIN);
    clampServo.attach(CLAMP_SERVO_PIN);

}

void loop() {

    if(updated){
      /* Parse V7RC data to servo timing (1000 to 2000)*/
      // CH1: front arm servo
      // CH2: rear arm servo
      // CH3: rotate servo
      // CH4: clamp servo
      int ch1=FRONT_SERVO_CENTER, ch2=REAR_SERVO_CENTER, ch3=ROTATE_SERVO_CENTER, ch4=CLAMP_SERVO_CENTER;
      char charBuff[3]; // for string to int function

      //Serial.println(rxString); // raw output
      if((rxString.substring(0,3).compareTo("SRV") == 0) || (rxString.substring(0,3).compareTo("SRT") == 0)){
        ch1 = rxString.substring(3,7).toInt();
        ch2 = rxString.substring(7,11).toInt();
        ch3 = rxString.substring(11,15).toInt();
        ch4 = rxString.substring(15,19).toInt();
      }
      else if(rxString.substring(0,3).compareTo("SS8") == 0){
        rxString.substring(3,5).toCharArray(charBuff,3);
        ch1 = strtol(charBuff,false,16)*10;
        rxString.substring(5,7).toCharArray(charBuff,3);
        ch2 = strtol(charBuff,false,16)*10;
        rxString.substring(7,9).toCharArray(charBuff,3);
        ch3 = strtol(charBuff,false,16)*10;
        rxString.substring(9,11).toCharArray(charBuff,3);
        ch4 = strtol(charBuff,false,16)*10;
      }

      /* Process servo data */
      if(ch1 < 1500 - FRONT_SERVO_DEADZONE)
        ch1 = map(ch1, 1000, 1500 - FRONT_SERVO_DEADZONE, FRONT_SERVO_MIN, FRONT_SERVO_CENTER);
      else if (ch1 > 1500 + FRONT_SERVO_DEADZONE)
        ch1 = map(ch1, 1500 + FRONT_SERVO_DEADZONE, 2000, FRONT_SERVO_CENTER, FRONT_SERVO_MAX);
      else ch1 = FRONT_SERVO_CENTER;

      if(ch2 < 1500 - REAR_SERVO_DEADZONE)
        ch2 = map(ch2, 1000, 1500 - REAR_SERVO_DEADZONE, REAR_SERVO_MIN, REAR_SERVO_CENTER);
      else if (ch2 > 1500 + REAR_SERVO_DEADZONE)
        ch2 = map(ch2, 1500 + REAR_SERVO_DEADZONE, 2000, REAR_SERVO_CENTER, REAR_SERVO_MAX);
      else ch2 = REAR_SERVO_CENTER;

      if(ch3 < 1500 - ROTATE_SERVO_DEADZONE)
        ch3 = map(ch3, 1000, 1500 - ROTATE_SERVO_DEADZONE, ROTATE_SERVO_MIN, ROTATE_SERVO_CENTER);
      else if (ch3 > 1500 + ROTATE_SERVO_DEADZONE)
        ch3 = map(ch3, 1500 + ROTATE_SERVO_DEADZONE, 2000, ROTATE_SERVO_CENTER, ROTATE_SERVO_MAX);
      else ch3 = ROTATE_SERVO_CENTER;

      if(ch4 < 1500 - CLAMP_SERVO_DEADZONE)
        ch4 = map(ch4, 1000, 1500 - CLAMP_SERVO_DEADZONE, CLAMP_SERVO_MIN, CLAMP_SERVO_CENTER);
      else if (ch4 > 1500 + CLAMP_SERVO_DEADZONE)
        ch4 = map(ch4, 1500 + CLAMP_SERVO_DEADZONE, 2000, CLAMP_SERVO_CENTER, CLAMP_SERVO_MAX);
      else ch4 = CLAMP_SERVO_CENTER;

      /* Print out servo data */

      Serial.print("CH1: ");
      Serial.print(ch1);
      Serial.print(", CH2: ");
      Serial.print(ch2);
      Serial.print(", CH3: ");
      Serial.print(ch3);
      Serial.print(", CH4: ");
      Serial.println(ch4);

      /* Output to servo */
      
      frontArmServo.writeMicroseconds(ch1);
      rearArmServo.writeMicroseconds(ch2);
      rotateServo.writeMicroseconds(ch3);
      clampServo.writeMicroseconds(ch4);
        
      updated = false; // clear flag for another update
    }
    delay(10); // slow down
}
