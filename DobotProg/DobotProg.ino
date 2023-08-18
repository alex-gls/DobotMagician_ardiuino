#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"
#include <TrackingCamDxl.h>
#include "DxlMaster.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

TrackingCamDxl trackingCam(1);
unsigned long previousMillis = 0; // stores last time cam was updated

bool gripper = true;
bool homepose = false;

static uint32_t timer = millis();
static uint32_t timer2 = millis();

EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

PTPCmd          PointTakeUp;
PTPCmd          PointTakeDown;

PTPCmd          PointStart;

PTPCmd          PointDropUp;
PTPCmd          PointDropDown;

PTPCmd          PointsUp[10];
PTPCmd          PointsDown[10];

uint64_t gQueuedCmdIndex;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();
  DxlMaster.begin(1000000);

  InitRAM();

  ProtocolInit();

  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);

  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);

  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);

  SetPTPCmd(&PointStart, true, &gQueuedCmdIndex);

  gEndEffectorParams.xBias = 0.0;
  gEndEffectorParams.yBias = 0.0;
  gEndEffectorParams.zBias = 0.0;

  SetEndEffectorParams(&gEndEffectorParams, true, &gQueuedCmdIndex);

  ProtocolProcess();
}

void Serialread()
{
  while (Serial3.available()) {
    uint8_t data = Serial3.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);

    }
  }
}

int Serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

void printf_begin(void)
{
  fdevopen( &Serial_putc, 0 );
}


void InitRAM(void)
{
  //Set JOG Model
  gJOGJointParams.velocity[0] = 100;
  gJOGJointParams.velocity[1] = 100;
  gJOGJointParams.velocity[2] = 100;
  gJOGJointParams.velocity[3] = 100;
  gJOGJointParams.acceleration[0] = 80;
  gJOGJointParams.acceleration[1] = 80;
  gJOGJointParams.acceleration[2] = 80;
  gJOGJointParams.acceleration[3] = 80;

  gJOGCoordinateParams.velocity[0] = 100;
  gJOGCoordinateParams.velocity[1] = 100;
  gJOGCoordinateParams.velocity[2] = 100;
  gJOGCoordinateParams.velocity[3] = 100;
  gJOGCoordinateParams.acceleration[0] = 80;
  gJOGCoordinateParams.acceleration[1] = 80;
  gJOGCoordinateParams.acceleration[2] = 80;
  gJOGCoordinateParams.acceleration[3] = 80;

  gJOGCommonParams.velocityRatio = 50;
  gJOGCommonParams.accelerationRatio = 50;

  gJOGCmd.cmd = AP_DOWN;
  gJOGCmd.isJoint = JOINT_MODEL;

  //Set PTP Model
  gPTPCoordinateParams.xyzVelocity = 100;
  gPTPCoordinateParams.rVelocity = 100;
  gPTPCoordinateParams.xyzAcceleration = 80;
  gPTPCoordinateParams.rAcceleration = 80;

  gPTPCommonParams.velocityRatio = 50;
  gPTPCommonParams.accelerationRatio = 50;

  gQueuedCmdIndex = 0;

  //                      POINTS SETTINGS
  int col = 5;
  int row = 2;
  int lengDown = 70;
  const int X = 275;
  const int Y = 135;
  const int Yminus = 43;
  const int Xminus = 51;
  int j = 0;
  int k = 0;

  for (int j = 0; j < row; j++) {
    for (int i = 0; i < col; i++) { 
      PointsUp[i].x = X - j * Xminus;
      PointsUp[i].y = Y - i * Yminus;
      PointsUp[i].z = -20;
      PointsUp[i].r = 0;
      PointsUp[i].ptpMode = MOVJ_XYZ;

      PointsDown[i].x = PointsUp[i].x;
      PointsDown[i].y = PointsUp[i].y;
      PointsDown[i].z = -70;
      PointsDown[i].r = PointsUp[i].r;
      PointsDown[i].ptpMode = MOVJ_XYZ;
    }
  }

  PointTakeUp.x = 275;
  PointTakeUp.y = -150;
  PointTakeUp.z = -20;
  PointTakeUp.r = 0;
  PointTakeUp.ptpMode = MOVJ_XYZ;

  PointTakeDown.x = PointTakeUp.x;
  PointTakeDown.y = PointTakeUp.y;
  PointTakeDown.z = -70;
  PointTakeDown.r = PointTakeUp.r;
  PointTakeDown.ptpMode = MOVJ_XYZ;

  PointDropUp.x = 130;
  PointDropUp.y = -155;
  PointDropUp.z = -20;
  PointDropUp.r = 0;
  PointDropUp.ptpMode = MOVJ_XYZ;

  PointDropDown.x = PointDropUp.x;
  PointDropDown.y = PointDropUp.y;
  PointDropDown.z = -35;
  PointDropDown.r = PointDropUp.r;
  PointDropDown.ptpMode = MOVJ_XYZ;

  PointStart.x = 150;
  PointStart.y = 0;
  PointStart.z = 0;
  PointStart.r = 0;
  PointStart.ptpMode = MOVJ_XYZ;

}

int mydelay = 0;

void loop()
{
  if (millis() - timer > 1000)
  {
    timer = millis();
    uint8_t n = trackingCam.readBlobs(5); // read data about 5 first objects

    Serial.print("Количество шариков: ");
    Serial.println(n);
    Serial.print("Тип: ");
    Serial.print(trackingCam.blob[0].type, DEC);
    //            Serial.print(" , X: ");
    //            Serial.print(trackingCam.blob[0].cx, DEC);
    //            Serial.print(" , Y: ");
    //            Serial.println(trackingCam.blob[0].cy, DEC);
    Serial.println("");
  }

  if (!homepose) {
    if (millis() - timer2 > 10000)
    {
      if (!homepose) {
        timer2 = millis();
        goToHome();
        delay(20000);
        homepose = true;
      }
    }
  }
  else {
    if (Serial.available()) {
      String message = Serial.readString();
      message.trim();

      Serial.print("Message: ");
      Serial.println(message);

      if (message == "Home" || message == "home") goToHome();

      if (message == "start") travelPos(PointStart);

      if (message == "tu") travelPos(PointTakeUp);
      if (message == "td") travelPos(PointTakeDown);
      if (message == "du") travelPos(PointDropUp);
      if (message == "dd") travelPos(PointDropDown);

      if (message == "u00") travelPos(PointsUp[0]);
      if (message == "u01") travelPos(PointsUp[1]);
      if (message == "u02") travelPos(PointsUp[2]);
      if (message == "u03") travelPos(PointsUp[3]);
      if (message == "u04") travelPos(PointsUp[4]);
      if (message == "u10") travelPos(PointsUp[5]);
      if (message == "u11") travelPos(PointsUp[6]);
      if (message == "u12") travelPos(PointsUp[7]);
      if (message == "u13") travelPos(PointsUp[8]);
      if (message == "u14") travelPos(PointsUp[9]);

      if (message == "d00") travelPos(PointsDown[0]);
      if (message == "d01") travelPos(PointsDown[1]);
      if (message == "d02") travelPos(PointsDown[2]);
      if (message == "d03") travelPos(PointsDown[3]);
      if (message == "d04") travelPos(PointsDown[4]);
      if (message == "d10") travelPos(PointsDown[5]);
      if (message == "d11") travelPos(PointsDown[6]);
      if (message == "d12") travelPos(PointsDown[7]);
      if (message == "d13") travelPos(PointsDown[8]);
      if (message == "d14") travelPos(PointsDown[9]);

      if (message == "g1") gripper = not gripper;

      ////////////////////GRIPPER/////////////////
      if (message == "On" || message == "on" || message == "ON") {
        OnOffGripper(true);
      }

      if (message == "Off" || message == "off" || message == "OFF") {
        OnOffGripper(false);
      }

      if (message == "mode1" || message == "Mode1") {
        mode1();
      }

      if (message == "mode2" || message == "Mode2") {
        mode2();
      }
    }
    mode1();
    mode2();
  }
}

void goToHome() {
  Message tempMessage;
  memset(&tempMessage, 0, sizeof(Message));
  tempMessage.id = 31;
  tempMessage.rw = true;
  tempMessage.isQueued = false;
  tempMessage.paramsLen = 0;
  memcpy(tempMessage.params, 0, tempMessage.paramsLen);
  MessageWrite(&gSerialProtocolHandler, &tempMessage);
  ProtocolProcess();
}

void takeBoll(int n) {
  travelPos(PointsUp[n]);
  delay(1000);
  travelPos(PointsDown[n]);
  delay(500);
  OnOffGripper(true);
  delay(200);
  travelPos(PointsUp[n]);
  delay(1000);
}

void dropBoll(int n) {
  travelPos(PointsUp[n]);
  delay(1000);
  travelPos(PointsDown[n]);
  delay(500);
  OnOffGripper(false);
  delay(200);
  travelPos(PointsUp[n]);
  delay(1000);
}

void takeBasket() {
  travelPos(PointTakeUp);
  delay(1000);
  travelPos(PointTakeDown);
  delay(500);
  OnOffGripper(true);
  delay(200);
  travelPos(PointTakeUp);
  delay(1000);
}

void dropBasket() {
  travelPos(PointDropUp);
  delay(1000);
  travelPos(PointDropDown);
  delay(500);
  OnOffGripper(false);
  delay(200);
  travelPos(PointDropUp);
  delay(1000);
}

void mode1()
{
  travelPos(PointStart);

  takeBoll(0);
  dropBoll(9);

  takeBoll(4);
  dropBoll(5);

  takeBoll(9);
  dropBoll(0);

  takeBoll(5);
  dropBoll(4);

  travelPos(PointStart);
}

void mode2() {
  travelPos(PointStart);

  takeBasket();
  dropBoll(0);

  takeBasket();
  dropBoll(2);

  takeBasket();
  dropBoll(4);

  takeBasket();
  dropBoll(6);

  takeBasket();
  dropBoll(8);

  takeBoll(0);
  dropBasket();

  takeBoll(2);
  dropBasket();

  takeBoll(4);
  dropBasket();

  takeBoll(6);
  dropBasket();

  takeBoll(8);
  dropBasket();

  travelPos(PointStart);
}

void travelPos(PTPCmd Point) {
  SetPTPCmd(&Point, true, &gQueuedCmdIndex);
  ProtocolProcess();
}

void OnOffGripper(bool OnOff) {
  if (gripper) {
    SetEndEffectorSuctionCup(OnOff, true, &gQueuedCmdIndex);
    ProtocolProcess();
    delay(1000);
  }
}
