#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include <SD.h>

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

const byte tx_Pin = 2;
const byte rx_Pin = 3;
const byte min_Limit_sw_Pin = 4;
const byte max_Limit_sw_Pin = 5;
const byte estop_sw_Pin = 7;
const byte ena_Pin = 21;
const byte dir_Pin = 19; // A2
const byte pul_Pin = 17; // A4
const byte foot_sw_Pin = 6;
const int chipSelect = SDCARD_SS_PIN;
static unsigned long last_interrupt = 0;
static unsigned long last_trigger = 0;
static unsigned long last_bounce = 0;
static unsigned long curr_time = 0;
const static unsigned long DELAY_BOUNCE = 20;
const static unsigned long DELAY_FSW_TRIGGER = 4000;
const char *DELIMITER = ",";
const char *NEWLINE = "\n";

bool minLimit_trigger = false;
bool maxLimit_trigger = false;
bool eStop_trigger = false;
bool zero_block = false;
bool max_block = false;
bool error_state = false;
bool stp_state = false;
bool fsw_enable = false;
bool fsw_prev_bounce_state = false;
bool fsw_prev_steady_state = false;
bool fsw_curr_state = false;
bool zeroed = false;

float volume = 250;
int steps = 0;
float speed = 0;
float vol_per_1600steps = 45.55;
float sdelay = 1000000;
bool dispensed = false;
float vol_per_min = 544.5;
float max_Vol = 501.00;
float max_delay = 10;
float max_rate = 4800;
float max_cal = 100;
int jogAmt = 100;
int mode = 1; // 1,2,3
int pos = 0;
int errcode = 0;

char buffer[100] = { 0 };
#ifdef nex_enable
NexPage page0 = NexPage( 0, 0, "page0" );
NexPage page1 = NexPage( 1, 0, "page1" );
NexPage page2 = NexPage( 2, 0, "page2" );
NexPage page3 = NexPage( 3, 0, "page3" );

// Page 0
NexButton bDisp = NexButton( 0, 5, "bDisp" );
NexButton bZero = NexButton( 0, 6, "bZero" );
NexButton bVol = NexButton( 0, 10, "bVol" );

NexPicture pError = NexPicture( 0, 11, "p0" );
NexPicture pStp_ena = NexPicture( 0, 4, "p1" );
NexPicture pStp_dis = NexPicture( 0, 3, "p2" );
NexPicture pFsw_ena = NexPicture( 0, 2, "p3" );
NexPicture pFsw_dis = NexPicture( 0, 1, "p4" );

NexVariable mode0 = NexVariable( 0, 12, "Main.mode0" );
NexVariable err0 = NexVariable( 0, 13, "Main.err0" );
NexVariable stp0 = NexVariable( 0, 14, "Main.stp0" );
NexVariable fsw0 = NexVariable( 0, 15, "Main.fsw0" );
NexVariable vol0 = NexVariable( 0, 17, "Main.vol0" );
NexVariable ml160 = NexVariable( 0, 18, "Main.ml16" );
NexVariable mlmin0 = NexVariable( 0, 19, "Main.mlmin" );
NexVariable maxvol0 = NexVariable( 0, 20, "Main.maxvol" );
NexVariable sdelay0 = NexVariable( 0, 21, "Main.sdelay" );
NexVariable errcode0 = NexVariable( 0, 22, "Main.errcode" );
NexVariable maxdelay0 = NexVariable( 0, 23, "Main.lmtdelay" );
NexVariable maxrate0 = NexVariable( 0, 24, "Main.lmtrate" );
NexVariable maxcal0 = NexVariable( 0, 25, "Main.lmtcal" );

// Page 1
NexButton bv11 = NexButton( 1, 15, "b11" );
NexButton bv16 = NexButton( 1, 17, "b16" );
NexProgressBar progressBar = NexProgressBar( 6, 4, "j0" );

// Page 2
NexButton bt1 = NexButton( 2, 8, "b4" );
NexButton bt5 = NexButton( 2, 9, "b5" );
NexButton bt10 = NexButton( 2, 10, "b6" );
NexButton btUP = NexButton( 2, 6, "b1" );
NexButton btDN = NexButton( 2, 7, "b2" );
NexPicture pStp_ena2 = NexPicture( 2, 1, "p0" );
NexPicture pStp_dis2 = NexPicture( 2, 2, "p1" );
NexText sdelayBox = NexText( 2, 13, "t2" );

// Page 3
NexButton bcSave = NexButton( 3, 7, "b3" );
NexText mlstepBox = NexText( 3, 2, "t3" );
NexText mlminBox = NexText( 3, 1, "t2" );
NexButton bcMaxvol = NexButton( 3, 12, "b2" );
NexText maxvolBox = NexText( 3, 10, "t4" );

// Page 5
NexButton rstCtl = NexButton( 5, 4, "b1" );
NexButton rstScn = NexButton( 5, 5, "b2" );

// Page 6
NexButton abortBtn = NexButton( 6, 2, "b0" );

NexTouch *nex_listen_list[] =
{
    &pError, &bVol, &bDisp, &bZero, &pStp_ena, &pStp_dis, &pFsw_ena, &pFsw_dis,
    &pStp_ena2, &pStp_dis2, &bcSave, &bcMaxvol, &rstCtl, &rstScn, &abortBtn,
    &bt1, &bt5, &bt10, &btUP, &btDN, &sdelayBox,
    &bv11, &bv16, &bVol, &mlstepBox, &mlminBox,
    &page0, &page1, &page2, &page3,
    NULL };
/*Uart Serial2 (&sercom2, rx_Pin, tx_Pin, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler()
{
 Serial1.IrqHandler();
}*/

void update_icons()
{

  sendCommand( "ref 0" );
}
#endif

void min_Limit()
{
  if ( millis() - last_interrupt > 5000 )
  {
    minLimit_trigger = true;
    if ( !zero_block )
    {
      error_state = true;
    }
  }
  last_interrupt = millis();
}
void max_Limit()
{
  if ( millis() - last_interrupt > 5000 )
  {
    maxLimit_trigger = true;
    if ( !max_block )
    {
      error_state = true;
    }
  }
  last_interrupt = millis();
}
void e_Stop()
{
  if ( millis() - last_interrupt > 5000 )
  {
    eStop_trigger = true;
    error_state = true;
  }
  last_interrupt = millis();
}

void enableEstop()
{
  attachInterrupt( digitalPinToInterrupt( estop_sw_Pin ), e_Stop, RISING );
}

void enableLimitSwitches()
{
  attachInterrupt( digitalPinToInterrupt( min_Limit_sw_Pin ), min_Limit, RISING );
  attachInterrupt( digitalPinToInterrupt( max_Limit_sw_Pin ), max_Limit, RISING );
}

void disableEstop()
{
  detachInterrupt( digitalPinToInterrupt( estop_sw_Pin ) );
}

void disableLimitSwitches()
{
  detachInterrupt( digitalPinToInterrupt( min_Limit_sw_Pin ) );
  detachInterrupt( digitalPinToInterrupt( max_Limit_sw_Pin ) );
}

void safetyEna()
{
  enableEstop();
  enableLimitSwitches();
  // Serial.println("interrupts enabled");
}
void safetyDis()
{
  disableEstop();
  disableLimitSwitches();
  // Serial.println("interrupts disabled");
}

bool validate()
{
  bool result = true;

  if ( volume > 505 )
  {
    result = false;
  }

  if ( vol_per_1600steps > 42 || vol_per_1600steps < 38 )
  {
    result = false;
  }

  if ( vol_per_min < 0 || vol_per_min > 4200 )
  {
    result = false;
  }

  return result;

}

void pulse( float pulsedelay )
{
  digitalWrite( pul_Pin, HIGH );
  delayMicroseconds( ( int )pulsedelay );
  digitalWrite( pul_Pin, LOW );
  delayMicroseconds( ( int )pulsedelay );
}

int moveLinearAccel( float startDelay, float endDelay, unsigned long accelTime)
{
  float accelRate = ( startDelay - endDelay ) / 10;
  float pulseDelay = startDelay;
  unsigned long currTime = millis();
  unsigned long prevTime = currTime;
  unsigned long endTime = currTime + accelTime;
  unsigned long accelDelay = accelTime / 10;
  int stepsTraversed = 0;

  if ( stp_state )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 1000 );

    while ( currTime < endTime )
    {
      if ( error_state || !stp_state )
      {
        sendCommand( "page 0" );
        return -1;
      }
      currTime = millis();
      if((currTime - prevTime) > accelDelay) {
        pulseDelay = pulseDelay - accelRate;
        prevTime = currTime;
      }
      if ( pulseDelay < endDelay )
      {
        pulseDelay = endDelay;
      }
      pulse( pulseDelay );
      stepsTraversed++;
    }

  }

  return stepsTraversed;

}

void setDirection( PinStatus direction )
{

  digitalWrite( dir_Pin, direction );
  delayMicroseconds( 1000 );

}

bool move( float pulsedelay, float steps )
{

  if ( stp_state )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 1000 );

    for ( int i = 0; i < steps; i++ )
    {
      if ( error_state || !stp_state )
      {
        sendCommand( "page 0" );
        return false;
      }
      pulse( pulsedelay );
    }

  }

  return true;
}

void move_to_zero()
{ // 454
  // Serial.println("Moving to zero");
  enableLimitSwitches();
  zero_block = true;
  if ( stp_state )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 1000 );

    setDirection( HIGH );

    while ( !minLimit_trigger )
    {
      if ( error_state || !stp_state )
      {
        zero_block = false;
        sendCommand( "page 0" );
        return;
      }
      // nexLoop(nex_listen_list);
      pulse( 454 );
    }

    setDirection( LOW );
    delayMicroseconds( 200000 );

    for ( int i = 0; i < 200; i++ )
    {
      if ( error_state || !stp_state )
      {
        zero_block = false;
        sendCommand( "page 0" );
        return;
      }
      // nexLoop(nex_listen_list);
      pulse( 454 );
    }
    pos = 0;
  }
  minLimit_trigger = false;
  zero_block = false;
  dispensed = false;
  disableLimitSwitches();
  zeroed = true;
  sendCommand( "page 0" );
}
void move_to_max()
{
  // Serial.println("Moving to zero");
  enableLimitSwitches();
  max_block = true;
  if ( stp_state )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 1000 );
    setDirection( LOW );

    while ( !maxLimit_trigger )
    {
      if ( error_state || !stp_state )
      {
        max_block = false;
        sendCommand( "page 0" );
        return;
      }
      // nexLoop(nex_listen_list);
      pulse( 454 );
      pos++;
    }

    setDirection( HIGH );
    delayMicroseconds( 200000 );

    for ( int i = 0; i < 200; i++ )
    {
      if ( error_state || !stp_state )
      {
        max_block = false;
        sendCommand( "page 0" );
        return;
      }
      // nexLoop(nex_listen_list);
      pulse( 454 );
      pos--;
    }
    // pos = 0;
  }
  maxLimit_trigger = false;
  max_block = false;
  dispensed = false;
  disableLimitSwitches();
  max_Vol = vol_per_1600steps / 1600 * pos;
  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", max_Vol );
  maxvol0.setText( buffer );
  delayMicroseconds( 200000 );
  move_to_zero();
  sendCommand( "page 3" );
}
void dispense()
{
  // Serial.println("Dispensing");
  // Serial.print("Volume(mL)   :   ");
  // Serial.println(volume);
  // Serial.print("mL/1600 Steps:   ");
  // Serial.println(vol_per_1600steps);
  // Serial.print("mL/min       :   ");
  // Serial.println(vol_per_min);
  int stepsTraversed = -1;
  if ( stp_state && zeroed )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 200 );

    // microseconds/step = mL/step * min/mL * 60000000 microseconds/min
    if ( validate() )
    {
      speed = ( ( ( vol_per_1600steps / 1600 ) / vol_per_min ) * 30000000 );

      // Serial.print("Speed : ");
      // Serial.println(speed);
      if ( volume < max_Vol && volume > 0 )
      {
        setDirection( LOW );

        steps = volume / ( vol_per_1600steps / 1600 );

        if ( move( speed, steps ) )
        {

          delayMicroseconds( sdelay );
          setDirection( HIGH );
          delayMicroseconds( sdelay );
          // Serial.println(sdelay);
          stepsTraversed = moveLinearAccel( speed * 4, speed, 2000 );
          if ( stepsTraversed > 0  && stepsTraversed < steps)
          {
            move( speed, ( steps - stepsTraversed ) );
          }
        }


      }
    }
  }
  dispensed = false;
  sendCommand( "page 0" );
}

void cal1600()
{
  // Serial.println("Dispensing cal1600");
  if ( stp_state )
  {
    digitalWrite( ena_Pin, HIGH );
    delayMicroseconds( 100 );

    setDirection( LOW );

    if ( move( 454, 1600 ) )
    {

      delayMicroseconds( sdelay );
      setDirection( HIGH );
      delayMicroseconds( sdelay );
      // Serial.println(sdelay);
      move( 454, 1600 );

    }
  }
  dispensed = false;
  sendCommand( "page 1" );
}
void rstMCU()
{
  NVIC_SystemReset();
}

#ifdef sd_enable
bool SD_Begin( void )
{
  // Serial.print("Initializing SD card...");
  if ( !SD.begin( chipSelect ) )
  {
    // Serial.println("Card failed, or not present");
    return ( 0 );
  }
  else
  {
    // Serial.println("card initialized.");
    return ( 1 );
  }
}
size_t readField( File *file, char *str, size_t size, const char *delim )
{
  char ch;
  size_t n = 0;
  while ( ( n + 1 ) < size && file->read( &ch, 1 ) == 1 )
  {
    // Delete CR.
    if ( ch == '\r' )
    {
      continue;
    }
    str[n++] = ch;
    if ( strchr( delim, ch ) )
    {
      break;
    }
  }
  str[n] = '\0';
  return n;
}
void SD_ReadSettings()
{
  float values[15];
  File csvFile;
  char filename[15];
  strcpy( filename, "settings.txt" );
  csvFile = SD.open( filename, FILE_READ );
  if ( !csvFile )
  {
    // Serial.print("error opening ");
    // Serial.println(filename);
    while ( 1 )
      ;
  }

  // Rewind the file for read.
  csvFile.seek( 0 );

  size_t n;     // Length of returned field with delimiter.
  char str[20]; // Must hold longest field with delimiter and zero byte.
  char *ptr;    // Test for valid field.

  // Read the file and store the data.

  n = readField( &csvFile, str, sizeof( str ), DELIMITER );
  if ( n == 0 )
  {
    // Serial.println("Too few lines");
  }
  values[0] = strtof( str, &ptr );
  if ( ptr == str )
  {
    // Serial.println("bad number");
  }
  for(int i = 1; i < 8; i++){
    readField( &csvFile, str, sizeof( str ), NEWLINE );
    n = readField( &csvFile, str, sizeof( str ), DELIMITER );
    if ( n == 0 )
    {
      // Serial.println("Too few lines");
    }
    values[i] = strtof( str, &ptr );
    if ( ptr == str )
    {
      // Serial.println("bad number");
    }

  }
  volume = values[0];
  vol_per_1600steps = values[1];
  vol_per_min = values[2];
  max_Vol = values[3];
  sdelay = values[4];
  max_delay = values[5];
  max_rate = values[6];
  max_cal = values[7];
  n = readField( &csvFile, str, sizeof( str ), NEWLINE );
  // Allow missing endl at eof.
  if ( str[n - 1] != '\n' && csvFile.available() )
  {
    // Serial.println("missing endl");
  }

  csvFile.close();
  // Serial.print("Volume(mL)             :   ");
  // Serial.println(volume);
  // Serial.print("mL/1600 Steps          :   ");
  // Serial.println(vol_per_1600steps);
  // Serial.print("dispense rate (mL/min) :   ");
  // Serial.println(vol_per_min);
  // Serial.print("max volume (mL)        :   ");
  // Serial.println(max_Vol);
  // Serial.print("delay (s)              :   ");
  // Serial.println(sdelay);
}
void SD_WriteSettings()
{
  // Serial.println("Saving Settings");
  File csvFile;
  char filename[15];
  strcpy( filename, "settings.txt" );
  // SD.remove(filename);
  csvFile = SD.open( filename, O_WRITE | O_TRUNC );
  if ( !csvFile )
  {
    // Serial.print("error opening ");
    // Serial.println(filename);
    while ( 1 )
      ;
  }

  // Rewind the file for write.
  csvFile.seek( 0 );
  csvFile.print( volume );
  csvFile.println( ",volume" );
  csvFile.print( vol_per_1600steps );
  csvFile.println( ",vol_per_1600" );
  csvFile.print( vol_per_min );
  csvFile.println( ",vol_per_min" );
  csvFile.print( max_Vol );
  csvFile.println( ",max_Vol" );
  csvFile.print( sdelay );
  csvFile.println( ",delay_s" );

  csvFile.print( max_delay );
  csvFile.println( ",max_delay" );
  csvFile.print( max_rate );
  csvFile.println( ",max_rate" );
  csvFile.print( max_cal );
  csvFile.println( ",max_cal" );

  csvFile.println( "0,logfile" );
  csvFile.close();
  // Serial.println("Settings Saved");
  // Serial.print("Volume(mL)             :   ");
  // Serial.println(volume);
  // Serial.print("mL/1600 Steps          :   ");
  // Serial.println(vol_per_1600steps);
  // Serial.print("dispense rate (mL/min) :   ");
  // Serial.println(vol_per_min);
  // Serial.print("max volume (mL)        :   ");
  // Serial.println(max_Vol);
  // Serial.print("delay (s)              :   ");
  // Serial.println(sdelay);
}
#endif
#ifdef nex_enable
void bVolPopCallback( void *ptr )
{
  // Serial.println("Mode 1");
  mode = 1;
}
void mlstepBoxPopCallback( void *ptr )
{
  // Serial.println("Mode 2");
  mode = 2;
}
void mlminBoxPopCallback( void *ptr )
{
  // Serial.println("Mode 3");
  mode = 3;
}
void sdelayBoxPopCallback( void *ptr )
{
  // Serial.println("Mode 4");
  mode = 4;
}
void perrorPopCallback( void *ptr )
{
  // Serial.println("Reset error states");
  error_state = false;
  minLimit_trigger = false;
  maxLimit_trigger = false;
  eStop_trigger = false;
  dispensed = false;
}

void bZeroPopCallback( void *ptr )
{
  move_to_zero();
}

void bDispPopCallback( void *ptr )
{
  dispense();
}

void bt1PopCallback( void *ptr )
{
  jogAmt = 400;
  // Serial.print("Jog set to ");
  // Serial.println(jogAmt);
}
void bt5PopCallback( void *ptr )
{
  jogAmt = 2000;
  // Serial.print("Jog set to ");
  // Serial.println(jogAmt);
}
void bt10PopCallback( void *ptr )
{
  jogAmt = 4000;
  // Serial.print("Jog set to ");
  // Serial.println(jogAmt);
}
void btUPPopCallback( void *ptr )
{
  // digitalWrite(ena_Pin, LOW);
  // delayMicroseconds(100);

  digitalWrite( dir_Pin, HIGH );
  delayMicroseconds( 100 );
  for ( int i = 0; i < jogAmt; i++ )
  {
    if ( error_state || !stp_state )
    {
      return;
    }
    digitalWrite( pul_Pin, HIGH );
    delayMicroseconds( 1000 );
    digitalWrite( pul_Pin, LOW );
    delayMicroseconds( 1000 );
  }
}

void btDNPopCallback( void *ptr )
{
  // digitalWrite(ena_Pin, HIGH);
  // delayMicroseconds(100);

  digitalWrite( dir_Pin, LOW );
  delayMicroseconds( 100 );
  for ( int i = 0; i < jogAmt; i++ )
  {
    if ( error_state || !stp_state )
    {
      return;
    }
    digitalWrite( pul_Pin, HIGH );
    delayMicroseconds( 1000 );
    digitalWrite( pul_Pin, LOW );
    delayMicroseconds( 1000 );
  }
}

void bv11PopCallback( void *ptr )
{
  //
  delayMicroseconds( 100 );
  switch ( mode )
  {
  case 1:
    memset( buffer, 0, sizeof( buffer ) );
    vol0.getText( buffer, sizeof( buffer ) );

    if ( atof( buffer ) < max_Vol )
    {
      volume = atof( buffer );
      // Serial.print("volume set to ");
      // Serial.println(volume);
    }
    else
    {
      // handle error
      memset( buffer, 0, sizeof( buffer ) );
      snprintf( buffer, sizeof( buffer ), "%.2f", volume );
      vol0.setText( buffer );
      errcode0.setValue( 5 );
      sendCommand( "page 7" );
    }

    break;
  case 2:
    memset( buffer, 0, sizeof( buffer ) );
    ml160.getText( buffer, sizeof( buffer ) );
    if ( atof( buffer ) < max_cal )
    {
      vol_per_1600steps = atof( buffer );
      // Serial.print("volume per 1600 steps set to ");
      // Serial.println(vol_per_1600steps);
    }
    else
    {
      // handle error
      memset( buffer, 0, sizeof( buffer ) );
      snprintf( buffer, sizeof( buffer ), "%.4f", vol_per_1600steps );
      ml160.setText( buffer );
      errcode0.setValue( 6 );
      sendCommand( "page 7" );
    }

    break;
  case 3:
    memset( buffer, 0, sizeof( buffer ) );
    mlmin0.getText( buffer, sizeof( buffer ) );
    if ( atof( buffer ) < max_rate )
    {
      vol_per_min = atof( buffer );
      // Serial.print("volume per min set to ");
      // Serial.println(vol_per_min);
    }
    else
    {
      // handle error
      memset( buffer, 0, sizeof( buffer ) );
      snprintf( buffer, sizeof( buffer ), "%.2f", vol_per_min );
      mlmin0.setText( buffer );
      errcode0.setValue( 7 );
      sendCommand( "page 7" );
    }

    break;
  case 4:
    memset( buffer, 0, sizeof( buffer ) );
    sdelay0.getText( buffer, sizeof( buffer ) );
    if ( atof( buffer ) < max_delay )
    {
      sdelay = atof( buffer ) * 1000000;
      // Serial.print("delay (s) set to ");
      // Serial.println(sdelay);
    }
    else
    {
      // handle error
      memset( buffer, 0, sizeof( buffer ) );
      snprintf( buffer, sizeof( buffer ), "%.6f", ( sdelay / 1000000 ) );
      sdelay0.setText( buffer );
      errcode0.setValue( 4 );
      sendCommand( "page 7" );
    }

    break;
    // default:
    //  code block
  }
}
void bv16PopCallback( void *ptr )
{
  cal1600();
}
void pStp_enaPopCallback( void *ptr )
{
  stp_state = false;
  digitalWrite( ena_Pin, LOW );
  // Serial.println("Stepper disabled");
}
void pStp_disPopCallback( void *ptr )
{
  stp_state = true;
  digitalWrite( ena_Pin, HIGH );
  // Serial.println("Stepper enabled");
}
void pFsw_enaPopCallback( void *ptr )
{
  fsw_enable = false;
  // Serial.println("Foot switch disabled");
}
void pFsw_disPopCallback( void *ptr )
{
  fsw_enable = true;
  // Serial.println("Foot switch enabled");
}
void pStp_ena2PopCallback( void *ptr )
{
  stp_state = false;
  digitalWrite( ena_Pin, LOW );
  // Serial.println("Stepper disabled");
}
void pStp_dis2PopCallback( void *ptr )
{
  stp_state = true;
  digitalWrite( ena_Pin, HIGH );
  // Serial.println("Stepper enabled");
}
void bcSavePopCallback( void *ptr )
{
  SD_WriteSettings();
}
void bcMaxvolPopCallback( void *ptr )
{
  move_to_max();
}
void rstScnPopCallback( void *ptr )
{
  delayMicroseconds( 30000000 );
  // Serial.println("nexinit");
  nexInit();
  delayMicroseconds( 5000000 );
  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", volume );
  vol0.setText( buffer );
  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.4f", vol_per_1600steps );
  ml160.setText( buffer );
  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", vol_per_min );
  mlmin0.setText( buffer );
  sendCommand( "page 0" );
}
void rstCtlPopCallback( void *ptr )
{
  rstMCU();
}
void abortBtnPopCallback( void *ptr )
{
  // Serial.println("Abort movement");
  digitalWrite( ena_Pin, LOW );
  stp_state = false;
  delayMicroseconds( 500000 );
  sendCommand( "page 0" );
}
#endif

void setup()
{
  Serial.begin( 9600 );
  for ( int i = 5; i > 0; i-- )
  {
    delayMicroseconds( 1000000 );
    // Serial.println(i);
  }

  pinMode( min_Limit_sw_Pin, INPUT );
  pinMode( max_Limit_sw_Pin, INPUT );
  pinMode( estop_sw_Pin, INPUT );

  pinMode( foot_sw_Pin, INPUT );

  pinMode( ena_Pin, OUTPUT );
  pinMode( dir_Pin, OUTPUT );
  pinMode( pul_Pin, OUTPUT );
  // pinMode(LED_BUILTIN, OUTPUT);
  delayMicroseconds( 1000000 );
  // Serial.println("pinmode setup complete");

#ifdef sd_enable
  SD_Begin();
  delayMicroseconds( 1000000 );
  // Serial.println("reading usb");
  for ( int i = 5; i > 0; i-- )
  {
    delayMicroseconds( 1000000 );
    // Serial.println(i);
  }
  SD_ReadSettings();

#endif
#ifdef nex_enable
  // Serial1.begin(9600);

  // pinPeripheral(tx_Pin, PIO_SERCOM);
  // pinPeripheral(rx_Pin, PIO_SERCOM_ALT);

  // Serial1.begin(9600);

  nexInit();
  delayMicroseconds( 2000000 );
  // Serial.println("nexinit");

  pError.attachPop( perrorPopCallback, &pError );
  bVol.attachPop( bVolPopCallback, &bVol );
  bDisp.attachPop( bDispPopCallback, &bDisp );
  bZero.attachPop( bZeroPopCallback, &bZero );
  pStp_ena.attachPop( pStp_enaPopCallback, &pStp_ena );
  pStp_dis.attachPop( pStp_disPopCallback, &pStp_dis );
  pFsw_ena.attachPop( pFsw_enaPopCallback, &pFsw_ena );
  pFsw_dis.attachPop( pFsw_disPopCallback, &pFsw_dis );

  mlstepBox.attachPop( mlstepBoxPopCallback, &mlstepBox );
  mlminBox.attachPop( mlminBoxPopCallback, &mlminBox );
  bcSave.attachPop( bcSavePopCallback, &bcSave );
  bcMaxvol.attachPop( bcMaxvolPopCallback, &bcMaxvol );

  bt1.attachPop( bt1PopCallback, &bt1 );
  bt5.attachPop( bt5PopCallback, &bt5 );
  bt10.attachPop( bt10PopCallback, &bt10 );
  btUP.attachPop( btUPPopCallback, &btUP );
  btDN.attachPop( btDNPopCallback, &btDN );
  pStp_ena2.attachPop( pStp_ena2PopCallback, &pStp_ena2 );
  pStp_dis2.attachPop( pStp_dis2PopCallback, &pStp_dis2 );
  sdelayBox.attachPop( sdelayBoxPopCallback, &sdelayBox );

  bv11.attachPop( bv11PopCallback, &bv11 );
  bv16.attachPop( bv16PopCallback, &bv16 );

  rstCtl.attachPop( rstCtlPopCallback, &rstCtl );
  rstScn.attachPop( rstScnPopCallback, &rstScn );
  abortBtn.attachPop( abortBtnPopCallback, &abortBtn );
#endif

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", volume );
  vol0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.4f", vol_per_1600steps );
  ml160.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", vol_per_min );
  mlmin0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", max_Vol );
  maxvol0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.6f", ( sdelay / 1000000 ) );
  sdelay0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.6f", max_delay );
  maxdelay0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.2f", max_rate );
  maxrate0.setText( buffer );

  memset( buffer, 0, sizeof( buffer ) );
  snprintf( buffer, sizeof( buffer ), "%.4f", max_cal );
  maxcal0.setText( buffer );

  sendCommand( "page 0" );
  enableEstop();
}

void loop()
{

#ifdef nex_enable
  nexLoop( nex_listen_list );
#endif

  if ( minLimit_trigger && !zero_block )
  {
    // Serial.println("min limit triggered");
    digitalWrite( ena_Pin, LOW );
    error_state = true;
    sendCommand( "Main.err0.val=1" );
    delayMicroseconds( 200000 );
    sendCommand( "Main.stp0.val=0" );
    delayMicroseconds( 200000 );
    errcode0.setValue( 1 );
    delayMicroseconds( 200000 );
    sendCommand( "page 7" );
    minLimit_trigger = false;
  }
  if ( maxLimit_trigger && !max_block )
  {
    // Serial.println("max limit triggered");
    digitalWrite( ena_Pin, LOW );
    error_state = true;
    sendCommand( "Main.err0.val=1" );
    delayMicroseconds( 200000 );
    sendCommand( "Main.stp0.val=0" );
    delayMicroseconds( 200000 );
    errcode0.setValue( 2 );
    delayMicroseconds( 200000 );
    sendCommand( "page 7" );
    maxLimit_trigger = false;
  }
  if ( eStop_trigger )
  {
    // Serial.println("estop limit triggered");
    digitalWrite( ena_Pin, LOW );
    error_state = true;
    sendCommand( "Main.err0.val=1" );
    delayMicroseconds( 200000 );
    sendCommand( "Main.stp0.val=0" );
    delayMicroseconds( 200000 );
    errcode0.setValue( 3 );
    delayMicroseconds( 200000 );
    sendCommand( "page 7" );
    eStop_trigger = false;
    zeroed = false;
  }
  if ( fsw_enable )
  {
    fsw_curr_state = !digitalRead( foot_sw_Pin );
    if ( fsw_curr_state != fsw_prev_bounce_state )
    {
      last_bounce = millis();
      fsw_prev_bounce_state = fsw_curr_state;
    }

    curr_time = millis();
    if ( ( curr_time - last_trigger ) > DELAY_FSW_TRIGGER )
    {
      if ( ( curr_time - last_bounce ) > DELAY_BOUNCE )
      {
        if ( !fsw_prev_steady_state && fsw_curr_state )
        {
          last_trigger = curr_time;
          // Serial.println("foot switch triggered");
          if ( !error_state || !dispensed )
          {
            dispensed = true;
            dispense();
          }
        }
        fsw_prev_steady_state = fsw_curr_state;
      }
    }
  }
}
