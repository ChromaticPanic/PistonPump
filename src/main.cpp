//-------------------------------------------------------------
// Copyright (C) Joseffus Santos 2021 All Rights Reserved
//
//
//
//
//
//
//-------------------------------------------------------------

//-------------------------------------------------------------
// TODO
// ADD delay to zero or pull back a bit to prevent a double press
//         and have the min limit get ignored due to debounce
//
// ADD delay to dispense and footswitch to prevent double presses
//         and or have the carriage return to minlimit every time
//
//
//
//-------------------------------------------------------------

#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include <SD.h>
#include <assert.h>

#define sd_enable
#define nex_enable

#ifdef nex_enable
#include "Nextion.h"
#include "NexButton.h"
#include "NexText.h"
#include "NexProgressBar.h"
#include "NexPicture.h"
//#include <SoftwareSerial.h>
#endif

//

#define DEBOUNCE_MIN 2000
#define DEBOUNCE_MAX 2000
#define DEBOUNCE_ESTOP 2000
#define DEBOUNCE_FSW 5000
#define DEBOUNCE_ZERO 2200
#define DEBOUNCE_DISP 5000

#define tx_Pin 2
#define rx_Pin 3
#define min_Limit_sw_Pin 4
#define max_Limit_sw_Pin 5
#define estop_sw_Pin 7
#define ena_Pin 21
#define dir_Pin 19 //A2
#define pul_Pin 17 //A4
#define foot_sw_Pin 6
#define chipSelect SDCARD_SS_PIN

static char NEWLINE = '\n';
static char COMMA = ',';

//
static unsigned long last_intmin = 0;
static unsigned long last_intmax = 0;
static unsigned long last_intestop = 0;
static unsigned long last_fsw = 0;
static unsigned long last_zero = 0;
static unsigned long last_max = 0;
static unsigned long last_disp = 0;

//
bool minLimit_trigger = false;
bool maxLimit_trigger = false;
bool eStop_trigger = false;
bool zero_block = false;
bool max_block = false;
bool error_state = false;
bool stp_state = false;
bool fsw_state = false;
bool dispensing = false;

struct params
{
  float volume;
  double vol_per_1600steps;
  double vol_per_min;
  float sdelay;
  float safe_speed;

  float accel_pull;
  float accel_push;
  float speed_pull;
  float speed_push;
  float accel_time;

  int steps;
  float speed;
  bool dispensed;
  int jogAmt;
  int mode;
  int pos;
};

struct params value;
struct params minAllowed;
struct params maxAllowed;

char buffer[100] = {0};

void trigger_check(void);
void set_Pins(void);
void move_to_zero(void);
void move_to_max(void);
void dispense(void);
void cal1600(void);
void min_Limit(void);
void max_Limit(void);
void e_Stop(void);
void rstMCU(void);
void safetyEna(void);
void safetyDis(void);
void setDefaults(void);
void step(int);
void printSettings(void);

#ifdef nex_enable
void bVolPopCallback(void *ptr);
void mlstepBoxPopCallback(void *ptr);
void mlminBoxPopCallback(void *ptr);
void sdelayBoxPopCallback(void *ptr);
void perrorPopCallback(void *ptr);
void bZeroPopCallback(void *ptr);
void bDispPopCallback(void *ptr);
void bt1PopCallback(void *ptr);
void bt5PopCallback(void *ptr);
void bt10PopCallback(void *ptr);
void btUPPopCallback(void *ptr);
void btDNPopCallback(void *ptr);
void bv11PopCallback(void *ptr);
void bv16PopCallback(void *ptr);
void pStp_enaPopCallback(void *ptr);
void pStp_disPopCallback(void *ptr);
void pFsw_enaPopCallback(void *ptr);
void pFsw_disPopCallback(void *ptr);
void pStp_ena2PopCallback(void *ptr);
void pStp_dis2PopCallback(void *ptr);
void bcSavePopCallback(void *ptr);
void bcMaxvolPopCallback(void *ptr);
void rstScnPopCallback(void *ptr);
void rstCtlPopCallback(void *ptr);
void abortBtnPopCallback(void *ptr);
void update_icons(void);
void attach_Callbacks(void);
void update_Values(void);

NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");
NexPage page3 = NexPage(3, 0, "page3");

//Page 0
NexButton bDisp = NexButton(0, 5, "bDisp");
NexButton bZero = NexButton(0, 6, "bZero");
NexButton bVol = NexButton(0, 10, "bVol");

NexPicture pError = NexPicture(0, 11, "p0");
NexPicture pStp_ena = NexPicture(0, 4, "p1");
NexPicture pStp_dis = NexPicture(0, 3, "p2");
NexPicture pFsw_ena = NexPicture(0, 2, "p3");
NexPicture pFsw_dis = NexPicture(0, 1, "p4");

NexVariable mode0 = NexVariable(0, 12, "Main.mode0");
NexVariable err0 = NexVariable(0, 13, "Main.err0");
NexVariable stp0 = NexVariable(0, 14, "Main.stp0");
NexVariable fsw0 = NexVariable(0, 15, "Main.fsw0");
NexVariable vol0 = NexVariable(0, 17, "Main.vol0");
NexVariable ml160 = NexVariable(0, 18, "Main.ml16");
NexVariable mlmin0 = NexVariable(0, 19, "Main.mlmin");
NexVariable maxvol0 = NexVariable(0, 20, "Main.maxvol");
NexVariable sdelay0 = NexVariable(0, 21, "Main.sdelay");
NexVariable errcode0 = NexVariable(0, 22, "Main.errcode");
NexVariable maxdelay0 = NexVariable(0, 23, "Main.lmtdelay");
NexVariable maxrate0 = NexVariable(0, 24, "Main.lmtrate");
NexVariable maxcal0 = NexVariable(0, 25, "Main.lmtcal");

//Page 1
NexButton bv11 = NexButton(1, 15, "b11");
NexButton bv16 = NexButton(1, 17, "b16");
NexProgressBar progressBar = NexProgressBar(6, 4, "j0");

//Page 2
NexButton bt1 = NexButton(2, 8, "b4");
NexButton bt5 = NexButton(2, 9, "b5");
NexButton bt10 = NexButton(2, 10, "b6");
NexButton btUP = NexButton(2, 6, "b1");
NexButton btDN = NexButton(2, 7, "b2");
NexPicture pStp_ena2 = NexPicture(2, 1, "p0");
NexPicture pStp_dis2 = NexPicture(2, 2, "p1");
NexText sdelayBox = NexText(2, 13, "t2");

//Page 3
NexButton bcSave = NexButton(3, 7, "b3");
NexText mlstepBox = NexText(3, 2, "t3");
NexText mlminBox = NexText(3, 1, "t2");
NexButton bcMaxvol = NexButton(3, 12, "b2");
NexText maxvolBox = NexText(3, 10, "t4");

//Page 5
NexButton rstCtl = NexButton(5, 4, "b1");
NexButton rstScn = NexButton(5, 5, "b2");

//Page 6
NexButton abortBtn = NexButton(6, 2, "b0");

NexTouch *nex_listen_list[] =
    {
        &pError, &bVol, &bDisp, &bZero, &pStp_ena, &pStp_dis, &pFsw_ena, &pFsw_dis,
        &pStp_ena2, &pStp_dis2, &bcSave, &bcMaxvol, &rstCtl, &rstScn, &abortBtn,
        &bt1, &bt5, &bt10, &btUP, &btDN, &sdelayBox,
        &bv11, &bv16, &bVol, &mlstepBox, &mlminBox,
        &page0, &page1, &page2, &page3,
        NULL};
/*Uart Serial2 (&sercom2, rx_Pin, tx_Pin, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler()
{
 Serial1.IrqHandler();
}*/
#endif

#ifdef sd_enable
bool SD_Begin(void);
size_t readField(File *, char *, size_t, char *);
void SD_ReadSettings(void);
float getValue(File *);
void SD_WriteSettings(void);
#endif

void setup(void)
{

  Serial.begin(9600);
  for (int i = 5; i > 0; i--)
  {
    delayMicroseconds(1000000);
  }

  set_Pins();
  delayMicroseconds(1000000);

  setDefaults();
  Serial.println("Default Settings:");
  printSettings();

#ifdef sd_enable
  SD_Begin();
  delayMicroseconds(1000000);
  //Serial.println("reading usb");
  for (int i = 5; i > 0; i--)
  {
    delayMicroseconds(1000000);
  }
  SD_ReadSettings();
  Serial.println("Loaded Settings:");
  printSettings();
#endif

#ifdef nex_enable
  nexInit();
  delayMicroseconds(2000000);
  attach_Callbacks();
  update_Values();
#endif

  safetyEna();
}

void loop(void)
{

#ifdef nex_enable
  nexLoop(nex_listen_list);
#endif

  trigger_check();
}

void setDefaults()
{
  value.volume = 250.0;
  value.vol_per_1600steps = 58.0800;
  value.vol_per_min = 544.5;
  value.sdelay = 1000000.0;
  value.safe_speed = 454.0;

  value.accel_pull = 20.0;
  value.accel_push = 20.0;
  value.speed_pull = 454.0;
  value.speed_push = 454.0;
  value.accel_time = 500.0;

  value.steps = 0;
  value.speed = 2000;
  value.dispensed = false;
  value.jogAmt = 100;
  value.mode = 1;
  value.pos = 0;

  minAllowed.volume = 0.0;
  minAllowed.vol_per_1600steps = 0.0;
  minAllowed.vol_per_min = 0.0;
  minAllowed.sdelay = 0.0;
  minAllowed.safe_speed = 300.0;

  minAllowed.accel_pull = 1.0;
  minAllowed.accel_push = 1.0;
  minAllowed.speed_pull = 1.0;
  minAllowed.speed_push = 1.0;
  minAllowed.accel_time = 9.0;

  minAllowed.steps = 0;
  minAllowed.speed = 0;
  minAllowed.jogAmt = 100;
  minAllowed.mode = 1;
  minAllowed.pos = 0;

  maxAllowed.volume = 600.0;
  maxAllowed.vol_per_1600steps = 100.0;
  maxAllowed.vol_per_min = 4800.0;
  maxAllowed.sdelay = 10.0;
  maxAllowed.safe_speed = 2000.0;

  maxAllowed.accel_pull = 1000.0;
  maxAllowed.accel_push = 1000.0;
  maxAllowed.speed_pull = 1000.0;
  maxAllowed.speed_push = 1000.0;
  maxAllowed.accel_time = 2001.0;

  maxAllowed.steps = 0;
  maxAllowed.speed = 0;
  maxAllowed.jogAmt = 100;
  maxAllowed.mode = 4;
  maxAllowed.pos = 0;
}
void set_Pins(void)
{
  pinMode(min_Limit_sw_Pin, INPUT);
  pinMode(max_Limit_sw_Pin, INPUT);
  pinMode(estop_sw_Pin, INPUT);
  pinMode(foot_sw_Pin, INPUT);
  pinMode(ena_Pin, OUTPUT);
  pinMode(dir_Pin, OUTPUT);
  pinMode(pul_Pin, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  //Serial.println("pinmode setup complete");
}
void update_Values(void)
{
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", value.volume);
  vol0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.4f", value.vol_per_1600steps);
  ml160.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", value.vol_per_min);
  mlmin0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", maxAllowed.volume);
  maxvol0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.6f", (value.sdelay / 1000000));
  sdelay0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.6f", maxAllowed.sdelay);
  maxdelay0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", maxAllowed.vol_per_min);
  maxrate0.setText(buffer);

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.4f", maxAllowed.vol_per_1600steps);
  maxcal0.setText(buffer);

  sendCommand("page 0");
}
void trigger_check(void)
{
  if (minLimit_trigger && !zero_block)
  {
    //Serial.println("min limit triggered");
    digitalWrite(ena_Pin, LOW);
    error_state = true;
    sendCommand("Main.err0.val=1");
    delayMicroseconds(200000);
    sendCommand("Main.stp0.val=0");
    delayMicroseconds(200000);
    errcode0.setValue(1);
    delayMicroseconds(200000);
    sendCommand("page 7");
    minLimit_trigger = false;
  }
  if (maxLimit_trigger && !max_block)
  {
    //Serial.println("max limit triggered");
    digitalWrite(ena_Pin, LOW);
    error_state = true;
    sendCommand("Main.err0.val=1");
    delayMicroseconds(200000);
    sendCommand("Main.stp0.val=0");
    delayMicroseconds(200000);
    errcode0.setValue(2);
    delayMicroseconds(200000);
    sendCommand("page 7");
    maxLimit_trigger = false;
  }
  if (eStop_trigger)
  {
    //Serial.println("estop limit triggered");
    digitalWrite(ena_Pin, LOW);
    error_state = true;
    sendCommand("Main.err0.val=1");
    delayMicroseconds(200000);
    sendCommand("Main.stp0.val=0");
    delayMicroseconds(200000);
    errcode0.setValue(3);
    delayMicroseconds(200000);
    sendCommand("page 7");
    eStop_trigger = false;
  }

  if (fsw_state && !digitalRead(foot_sw_Pin))
  {
    if (((millis() - last_fsw) > DEBOUNCE_FSW) && !dispensing)
    {
      last_fsw = millis();
      //Serial.println("foot switch triggered");
      if (!error_state || !value.dispensed)
      {
        value.dispensed = true;
        dispense();
      }
    }
  }
}
void step(int delay)
{
  digitalWrite(pul_Pin, HIGH);
  delayMicroseconds(delay);
  digitalWrite(pul_Pin, LOW);
  delayMicroseconds(delay);
}

void move_to_zero(void)
{ //454
  //Serial.println("Moving to zero");
  if (millis() - last_zero > DEBOUNCE_ZERO)
  {
    last_zero = millis();
    if (stp_state)
    {
      digitalWrite(ena_Pin, HIGH);
      delayMicroseconds(100);

      digitalWrite(dir_Pin, LOW);
      delayMicroseconds(200000);

      for (int i = 0; i < 2000; i++)
      {
        if (error_state || !stp_state)
        {
          zero_block = false;
          sendCommand("page 0");
          return;
        }
        //nexLoop(nex_listen_list);
        step(value.safe_speed);
      }

      zero_block = true;

      digitalWrite(dir_Pin, HIGH);
      delayMicroseconds(100);

      while (!minLimit_trigger)
      {
        if (error_state || !stp_state)
        {
          zero_block = false;
          sendCommand("page 0");
          return;
        }
        //nexLoop(nex_listen_list);
        step(value.safe_speed);
      }

      digitalWrite(dir_Pin, LOW);
      delayMicroseconds(200000);

      for (int i = 0; i < 200; i++)
      {
        if (error_state || !stp_state)
        {
          zero_block = false;
          sendCommand("page 0");
          return;
        }
        //nexLoop(nex_listen_list);
        step(value.safe_speed);
      }
      value.pos = 0;
    }
    minLimit_trigger = false;
    zero_block = false;
    value.dispensed = false;
    sendCommand("page 0");
  }
}
void move_to_max(void)
{
  //Serial.println("Moving to zero");
  max_block = true;
  if (stp_state)
  {
    digitalWrite(ena_Pin, HIGH);
    delayMicroseconds(100);

    digitalWrite(dir_Pin, LOW);
    delayMicroseconds(100);

    while (!maxLimit_trigger)
    {
      if (error_state || !stp_state)
      {
        max_block = false;
        sendCommand("page 0");
        return;
      }
      //nexLoop(nex_listen_list);
      step(value.safe_speed);
      value.pos++;
    }

    digitalWrite(dir_Pin, HIGH);
    delayMicroseconds(200000);

    for (int i = 0; i < 200; i++)
    {
      if (error_state || !stp_state)
      {
        max_block = false;
        sendCommand("page 0");
        return;
      }
      //nexLoop(nex_listen_list);
      step(value.safe_speed);
      value.pos--;
    }
    //pos = 0;
  }
  maxLimit_trigger = false;
  max_block = false;
  value.dispensed = false;
  maxAllowed.volume = value.vol_per_1600steps / 1600 * value.pos;
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", maxAllowed.volume);
  maxvol0.setText(buffer);
  delayMicroseconds(200000);
  move_to_zero();
  sendCommand("page 3");
}

void dispense(void)
{
  dispensing = true;
  unsigned long last_incr = 0;
  float currSpeed = value.safe_speed;
  if (millis() - last_disp > DEBOUNCE_DISP)
  {
    last_disp = millis();
    if (stp_state)
    {
      digitalWrite(ena_Pin, HIGH);
      delayMicroseconds(100);

      // microseconds/step = mL/step * min/mL * 60000000 microseconds/min

      if (value.volume < maxAllowed.volume && value.volume > 0) //if volume invalid do nothing
      {

        digitalWrite(dir_Pin, LOW);
        delayMicroseconds(100);

        value.steps = value.volume / (value.vol_per_1600steps / 1600);

        currSpeed = value.safe_speed;
        Serial.print("Curr Speed : ");
        Serial.println(currSpeed);
        // Draw water . the pullback speed can only be modified through the sd card value
        for (int i = 0; i < value.steps; i++)
        {
          if (error_state || !stp_state)
          {
            sendCommand("page 0");
            return;
          }
          //nexLoop(nex_listen_list);
          if ((millis() - last_incr > value.accel_time) && currSpeed > value.speed_pull)
          { // currspeed starts at the safe speed and decrements towards the target
            currSpeed = currSpeed - value.accel_pull;
            last_incr = millis();
          }
          step(currSpeed);
        }
        Serial.print("Curr Speed : ");
        Serial.println(currSpeed);

        digitalWrite(dir_Pin, HIGH);
        delayMicroseconds(value.sdelay);
        //Serial.println(sdelay);

        currSpeed = value.safe_speed; // currspeed starts at the safe speed and decrements towards the target
        value.speed = (((value.vol_per_1600steps / 1600) / value.vol_per_min) * 30000000);
        Serial.print("Disp Speed : ");
        Serial.println(value.speed);
        value.speed_push = value.speed;
        Serial.print("Curr Speed : ");
        Serial.println(currSpeed);

        // Push out the water. the dispense speed can be set via HMI or through sd card value
        for (int i = 0; i < value.steps; i++)
        {
          if (error_state || !stp_state)
          {
            sendCommand("page 0");
            return;
          }
          //nexLoop(nex_listen_list);
          if ((millis() - last_incr > value.accel_time) && currSpeed > value.speed_push)
          {
            currSpeed = currSpeed - value.accel_push;
            last_incr = millis();
          }
          //step(value.speed);
          step(currSpeed);
        }
        Serial.print("Curr Speed : ");
        Serial.println(currSpeed);
      }
    }
  }
  dispensing = false;
  value.dispensed = false;
  sendCommand("page 0");
}
void cal1600(void)
{
  //Serial.println("Dispensing cal1600");
  if (stp_state)
  {
    digitalWrite(ena_Pin, HIGH);
    delayMicroseconds(100);

    digitalWrite(dir_Pin, LOW);
    delayMicroseconds(100);

    for (int i = 0; i < 1600; i++)
    {
      if (error_state || !stp_state)
      {
        sendCommand("page 0");
        return;
      }
      //nexLoop(nex_listen_list);
      step(value.safe_speed);
    }

    digitalWrite(dir_Pin, HIGH);
    delayMicroseconds(value.sdelay);

    for (int i = 0; i < 1600; i++)
    {
      if (error_state || !stp_state)
      {
        sendCommand("page 0");
        return;
      }
      //nexLoop(nex_listen_list);
      step(value.safe_speed);
    }
  }
  value.dispensed = false;
  sendCommand("page 1");
}
void min_Limit(void)
{
  if (millis() - last_intmin > DEBOUNCE_MIN)
  {
    minLimit_trigger = true;
    if (!zero_block)
    {
      error_state = true;
    }
  }
  last_intmin = millis();
}
void max_Limit(void)
{
  if (millis() - last_intmax > DEBOUNCE_MAX)
  {
    maxLimit_trigger = true;
    if (!max_block)
    {
      error_state = true;
    }
  }
  last_intmax = millis();
}
void e_Stop(void)
{
  if (millis() - last_intestop > DEBOUNCE_ESTOP)
  {
    eStop_trigger = true;
    error_state = true;
  }
  last_intestop = millis();
}
void rstMCU(void)
{
  NVIC_SystemReset();
}
void safetyEna(void)
{
  attachInterrupt(digitalPinToInterrupt(min_Limit_sw_Pin), min_Limit, RISING);
  attachInterrupt(digitalPinToInterrupt(max_Limit_sw_Pin), max_Limit, RISING);
  attachInterrupt(digitalPinToInterrupt(estop_sw_Pin), e_Stop, RISING);
  //Serial.println("interrupts enabled");
}
void safetyDis(void)
{
  detachInterrupt(digitalPinToInterrupt(min_Limit_sw_Pin));
  detachInterrupt(digitalPinToInterrupt(max_Limit_sw_Pin));
  detachInterrupt(digitalPinToInterrupt(estop_sw_Pin));
  //Serial.println("interrupts disabled");
}
void printSettings(void)
{
  Serial.print(value.volume);
  Serial.println(",volume");
  Serial.print(value.vol_per_1600steps);
  Serial.println(",vol_per_1600");
  Serial.print(value.vol_per_min);
  Serial.println(",vol_per_min");
  Serial.print(maxAllowed.volume);
  Serial.println(",max_Vol");
  Serial.print(value.sdelay);
  Serial.println(",delay_s");

  Serial.print(maxAllowed.sdelay);
  Serial.println(",max_delay");
  Serial.print(maxAllowed.vol_per_min);
  Serial.println(",max_rate");
  Serial.print(maxAllowed.vol_per_1600steps);
  Serial.println(",max_cal");

  Serial.print(value.accel_pull);
  Serial.println(",accel_pull");
  Serial.print(value.accel_push);
  Serial.println(",accel_push");
  Serial.print(value.speed_pull);
  Serial.println(",speed_pull");
  Serial.print(value.speed_push);
  Serial.println(",speed_push");
  Serial.print(value.accel_time);
  Serial.println(",accel_time");
}

#ifdef sd_enable
bool SD_Begin(void)
{
  //Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect))
  {
    //Serial.println("Card failed, or not present");
    return (0);
  }
  else
  {
    //Serial.println("card initialized.");
    return (1);
  }
}
size_t readField(File *file, char *str, size_t size, char *delim)
{
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1)
  {
    if (ch == '\r')
    {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch))
    {
      break;
    }
  }
  str[n] = '\0';
  return n;
}
void SD_ReadSettings(void)
{

  float volume;
  double vol_per_1600steps;
  double vol_per_min;
  float sdelay;
  float safe_speed;

  double max_vol1600;
  double max_volmin;
  float max_vol;

  float accel_pull;
  float accel_push;
  float speed_pull;
  float speed_push;
  float accel_time;

  File csvFile;
  char filename[50];
  strcpy(filename, "settings.txt");
  csvFile = SD.open(filename, FILE_READ);
  if (!csvFile)
  {
    //Serial.print("error opening ");
    //Serial.println(filename);
    while (1)
      ;
  }

  // Rewind the file for read.
  csvFile.seek(0);

  // Read the file and store the data.
  volume = getValue(&csvFile);
  vol_per_1600steps = getValue(&csvFile);
  vol_per_min = getValue(&csvFile);
  sdelay = getValue(&csvFile);
  safe_speed = getValue(&csvFile);

  max_vol = getValue(&csvFile);     //maxAllowed.volume
  max_volmin = getValue(&csvFile);  //maxAllowed.vol_per_min
  max_vol1600 = getValue(&csvFile); //maxAllowed.vol_per_1600steps

  accel_pull = getValue(&csvFile);
  accel_push = getValue(&csvFile);
  speed_pull = getValue(&csvFile);
  speed_push = getValue(&csvFile);
  accel_time = getValue(&csvFile);

  if (volume > minAllowed.volume && volume < maxAllowed.volume)
  {
    value.volume = volume;
  }
  if (vol_per_1600steps > minAllowed.vol_per_1600steps && vol_per_1600steps < maxAllowed.vol_per_1600steps)
  {
    value.vol_per_1600steps = vol_per_1600steps;
  }
  if (vol_per_min > minAllowed.vol_per_min && vol_per_min < maxAllowed.vol_per_min)
  {
    value.vol_per_min = vol_per_min;
  }
  if (sdelay > minAllowed.sdelay && sdelay < maxAllowed.sdelay)
  {
    value.sdelay = sdelay;
  }
  if (safe_speed > minAllowed.safe_speed && safe_speed < maxAllowed.safe_speed)
  {
    value.safe_speed = safe_speed;
  }

  if (max_vol > minAllowed.volume)
  {
    maxAllowed.volume = max_vol;
  }
  if (max_volmin > minAllowed.vol_per_min)
  {
    maxAllowed.vol_per_min = max_volmin;
  }
  if (max_vol1600 > minAllowed.vol_per_1600steps)
  {
    maxAllowed.vol_per_1600steps = max_vol1600;
  }

  if (accel_pull > minAllowed.accel_pull && accel_pull < maxAllowed.accel_pull)
  {
    value.accel_pull = accel_pull;
  }
  if (accel_push > minAllowed.accel_push && accel_push < maxAllowed.accel_push)
  {
    value.accel_push = accel_push;
  }
  if (speed_pull > minAllowed.speed_pull && speed_pull < maxAllowed.speed_pull)
  {
    value.speed_pull = speed_pull;
  }
  if (speed_push > minAllowed.speed_push && speed_push < maxAllowed.speed_push)
  {
    value.speed_push = speed_push;
  }
  if (accel_time > minAllowed.accel_time && accel_time < maxAllowed.accel_time)
  {
    value.accel_time = accel_time;
  }

  csvFile.close();
}

float getValue(File *csvFile)
{
  float result = 0.0;
  //size_t n;      // Length of returned field with delimiter.
  char str[100]; // Must hold longest field with delimiter and zero byte.
  char *ptr;     // Test for valid field.

  if (readField(csvFile, str, sizeof(str), &COMMA) > 0)
  {
    if (str > 0)
    {
      result = strtof(str, &ptr);
    } //Serial.println("bad number");

  } //Serial.println("Too few lines");

  readField(csvFile, str, sizeof(str), &NEWLINE);

  return result;
}
void SD_WriteSettings(void)
{
  //Serial.println("Saving Settings");
  File csvFile;
  char filename[15];
  strcpy(filename, "settings.txt");
  //SD.remove(filename);
  csvFile = SD.open(filename, O_WRITE | O_TRUNC);
  if (!csvFile)
  {
    //Serial.print("error opening ");
    //Serial.println(filename);
    while (1)
      ;
  }

  // Rewind the file for write.
  csvFile.seek(0);
  csvFile.print(value.volume);
  csvFile.println(",volume");
  csvFile.print(value.vol_per_1600steps);
  csvFile.println(",vol_per_1600");
  csvFile.print(value.vol_per_min);
  csvFile.println(",vol_per_min");
  csvFile.print(value.sdelay);
  csvFile.println(",delay_s");
  csvFile.print(value.safe_speed);
  csvFile.println(",safe_speed");

  csvFile.print(maxAllowed.volume);
  csvFile.println(",max_Vol");
  csvFile.print(maxAllowed.vol_per_min);
  csvFile.println(",max_rate");
  csvFile.print(maxAllowed.vol_per_1600steps);
  csvFile.println(",max_cal");

  csvFile.print(value.accel_pull);
  csvFile.println(",accel_pull");
  csvFile.print(value.accel_push);
  csvFile.println(",accel_push");
  csvFile.print(value.speed_pull);
  csvFile.println(",speed_pull");
  csvFile.print(value.speed_push);
  csvFile.println(",speed_push");
  csvFile.print(value.accel_time);
  csvFile.println(",accel_time");

  csvFile.println("0,logfile");
  csvFile.close();
}
#endif

#ifdef nex_enable
void attach_Callbacks(void)
{
  pError.attachPop(perrorPopCallback, &pError);
  bVol.attachPop(bVolPopCallback, &bVol);
  bDisp.attachPop(bDispPopCallback, &bDisp);
  bZero.attachPop(bZeroPopCallback, &bZero);
  pStp_ena.attachPop(pStp_enaPopCallback, &pStp_ena);
  pStp_dis.attachPop(pStp_disPopCallback, &pStp_dis);
  pFsw_ena.attachPop(pFsw_enaPopCallback, &pFsw_ena);
  pFsw_dis.attachPop(pFsw_disPopCallback, &pFsw_dis);
  mlstepBox.attachPop(mlstepBoxPopCallback, &mlstepBox);
  mlminBox.attachPop(mlminBoxPopCallback, &mlminBox);
  bcSave.attachPop(bcSavePopCallback, &bcSave);
  bcMaxvol.attachPop(bcMaxvolPopCallback, &bcMaxvol);
  bt1.attachPop(bt1PopCallback, &bt1);
  bt5.attachPop(bt5PopCallback, &bt5);
  bt10.attachPop(bt10PopCallback, &bt10);
  btUP.attachPop(btUPPopCallback, &btUP);
  btDN.attachPop(btDNPopCallback, &btDN);
  pStp_ena2.attachPop(pStp_ena2PopCallback, &pStp_ena2);
  pStp_dis2.attachPop(pStp_dis2PopCallback, &pStp_dis2);
  sdelayBox.attachPop(sdelayBoxPopCallback, &sdelayBox);
  bv11.attachPop(bv11PopCallback, &bv11);
  bv16.attachPop(bv16PopCallback, &bv16);
  rstCtl.attachPop(rstCtlPopCallback, &rstCtl);
  rstScn.attachPop(rstScnPopCallback, &rstScn);
  abortBtn.attachPop(abortBtnPopCallback, &abortBtn);
}
void bVolPopCallback(void *ptr)
{
  //Serial.println("Mode 1");
  value.mode = 1;
}
void mlstepBoxPopCallback(void *ptr)
{
  //Serial.println("Mode 2");
  value.mode = 2;
}
void mlminBoxPopCallback(void *ptr)
{
  //Serial.println("Mode 3");
  value.mode = 3;
}
void sdelayBoxPopCallback(void *ptr)
{
  //Serial.println("Mode 4");
  value.mode = 4;
}
void perrorPopCallback(void *ptr)
{
  //Serial.println("Reset error states");
  error_state = false;
  minLimit_trigger = false;
  maxLimit_trigger = false;
  eStop_trigger = false;
  value.dispensed = false;
}
void bZeroPopCallback(void *ptr)
{
  move_to_zero();
}
void bDispPopCallback(void *ptr)
{
  dispense();
}
void bt1PopCallback(void *ptr)
{
  value.jogAmt = 400;
  //Serial.print("Jog set to ");
  //Serial.println(jogAmt);
}
void bt5PopCallback(void *ptr)
{
  value.jogAmt = 2000;
  //Serial.print("Jog set to ");
  //Serial.println(jogAmt);
}
void bt10PopCallback(void *ptr)
{
  value.jogAmt = 4000;
  //Serial.print("Jog set to ");
  //Serial.println(jogAmt);
}
void btUPPopCallback(void *ptr)
{
  //digitalWrite(ena_Pin, LOW);
  //delayMicroseconds(100);

  digitalWrite(dir_Pin, HIGH);
  delayMicroseconds(100);
  for (int i = 0; i < value.jogAmt; i++)
  {
    if (error_state || !stp_state)
    {
      return;
    }
    step(value.safe_speed * 2);
  }
}
void btDNPopCallback(void *ptr)
{
  //digitalWrite(ena_Pin, HIGH);
  //delayMicroseconds(100);

  digitalWrite(dir_Pin, LOW);
  delayMicroseconds(100);
  for (int i = 0; i < value.jogAmt; i++)
  {
    if (error_state || !stp_state)
    {
      return;
    }
    step(value.safe_speed * 2);
  }
}
void bv11PopCallback(void *ptr)
{
  //
  delayMicroseconds(100);
  switch (value.mode)
  {
  case 1:
    memset(buffer, 0, sizeof(buffer));
    vol0.getText(buffer, sizeof(buffer));

    if (atof(buffer) < maxAllowed.volume)
    {
      value.volume = atof(buffer);
      //Serial.print("volume set to ");
      //Serial.println(volume);
    }
    else
    {
      //handle error
      memset(buffer, 0, sizeof(buffer));
      snprintf(buffer, sizeof(buffer), "%.2f", value.volume);
      vol0.setText(buffer);
      errcode0.setValue(5);
      sendCommand("page 7");
    }

    break;
  case 2:
    memset(buffer, 0, sizeof(buffer));
    ml160.getText(buffer, sizeof(buffer));
    if (atof(buffer) < maxAllowed.vol_per_1600steps)
    {
      value.vol_per_1600steps = atof(buffer);
      //Serial.print("volume per 1600 steps set to ");
      //Serial.println(vol_per_1600steps);
    }
    else
    {
      //handle error
      memset(buffer, 0, sizeof(buffer));
      snprintf(buffer, sizeof(buffer), "%.4f", value.vol_per_1600steps);
      ml160.setText(buffer);
      errcode0.setValue(6);
      sendCommand("page 7");
    }

    break;
  case 3:
    memset(buffer, 0, sizeof(buffer));
    mlmin0.getText(buffer, sizeof(buffer));
    if (atof(buffer) < maxAllowed.vol_per_min)
    {
      value.vol_per_min = atof(buffer);
      //Serial.print("volume per min set to ");
      //Serial.println(vol_per_min);
    }
    else
    {
      //handle error
      memset(buffer, 0, sizeof(buffer));
      snprintf(buffer, sizeof(buffer), "%.2f", value.vol_per_min);
      mlmin0.setText(buffer);
      errcode0.setValue(7);
      sendCommand("page 7");
    }

    break;
  case 4:
    memset(buffer, 0, sizeof(buffer));
    sdelay0.getText(buffer, sizeof(buffer));
    if (atof(buffer) < maxAllowed.sdelay)
    {
      value.sdelay = atof(buffer) * 1000000;
      //Serial.print("delay (s) set to ");
      //Serial.println(sdelay);
    }
    else
    {
      //handle error
      memset(buffer, 0, sizeof(buffer));
      snprintf(buffer, sizeof(buffer), "%.6f", (value.sdelay / 1000000));
      sdelay0.setText(buffer);
      errcode0.setValue(4);
      sendCommand("page 7");
    }

    break;
    //default:
    // code block
  }
}
void bv16PopCallback(void *ptr)
{
  cal1600();
}
void pStp_enaPopCallback(void *ptr)
{
  stp_state = false;
  digitalWrite(ena_Pin, LOW);
  //Serial.println("Stepper disabled");
}
void pStp_disPopCallback(void *ptr)
{
  stp_state = true;
  digitalWrite(ena_Pin, HIGH);
  //Serial.println("Stepper enabled");
}
void pFsw_enaPopCallback(void *ptr)
{
  fsw_state = false;
  //Serial.println("Foot switch disabled");
}
void pFsw_disPopCallback(void *ptr)
{
  fsw_state = true;
  //Serial.println("Foot switch enabled");
}
void pStp_ena2PopCallback(void *ptr)
{
  stp_state = false;
  digitalWrite(ena_Pin, LOW);
  //Serial.println("Stepper disabled");
}
void pStp_dis2PopCallback(void *ptr)
{
  stp_state = true;
  digitalWrite(ena_Pin, HIGH);
  //Serial.println("Stepper enabled");
}
void bcSavePopCallback(void *ptr)
{
  SD_WriteSettings();
}
void bcMaxvolPopCallback(void *ptr)
{
  move_to_max();
}
void rstScnPopCallback(void *ptr)
{
  delayMicroseconds(30000000);
  //Serial.println("nexinit");
  nexInit();
  delayMicroseconds(5000000);
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", value.volume);
  vol0.setText(buffer);
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.4f", value.vol_per_1600steps);
  ml160.setText(buffer);
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%.2f", value.vol_per_min);
  mlmin0.setText(buffer);
  sendCommand("page 0");
}
void rstCtlPopCallback(void *ptr)
{
  rstMCU();
}
void abortBtnPopCallback(void *ptr)
{
  //Serial.println("Abort movement");
  digitalWrite(ena_Pin, LOW);
  stp_state = false;
  delayMicroseconds(500000);
  sendCommand("page 0");
}
void update_icons(void)
{

  sendCommand("ref 0");
}
#endif