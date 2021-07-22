













//
const byte tx_Pin = 2;
const byte rx_Pin = 3;
const byte min_Limit_sw_Pin = 4;
const byte max_Limit_sw_Pin = 5;
const byte estop_sw_Pin = 7;
const byte ena_Pin = 21;
const byte dir_Pin = 19; //A2
const byte pul_Pin = 17; //A4
const byte foot_sw_Pin = 6;
const int chipSelect = SDCARD_SS_PIN;

static unsigned long last_interrupt = 0;
static unsigned long last_fsw = 0;
static char NEWLINE = '\n';
static char COMMA = ',';

//
bool minLimit_trigger = false;
bool maxLimit_trigger = false;
bool eStop_trigger = false;
bool zero_block = false;
bool max_block = false;
bool error_state = false;
bool stp_state = false;
bool fsw_state = false;

struct params{
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











void step(int delay){
  digitalWrite(pul_Pin, HIGH);
  delayMicroseconds(delay);
  digitalWrite(pul_Pin, LOW);
  delayMicroseconds(delay);
}

void move_to_zero(void){ //454
 //Serial.println("Moving to zero");
  zero_block = true;
  if(stp_state){
   digitalWrite(ena_Pin, HIGH);
   delayMicroseconds(100);
   
   digitalWrite(dir_Pin, HIGH);
   delayMicroseconds(100);

   while(!minLimit_trigger){
    if(error_state || !stp_state){
     zero_block = false;
     sendCommand("page 0");
     return;
    }
    //nexLoop(nex_listen_list);
    step(value.safe_speed);
   }

  digitalWrite(dir_Pin, LOW);
  delayMicroseconds(200000);

  for (int i = 0; i < 200; i++){
   if(error_state || !stp_state){
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
void move_to_max(void){
 //Serial.println("Moving to zero");
  max_block = true;
  if(stp_state){
   digitalWrite(ena_Pin, HIGH);
   delayMicroseconds(100);
   
   digitalWrite(dir_Pin, LOW);
   delayMicroseconds(100);

   while(!maxLimit_trigger){
    if(error_state || !stp_state){
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

  for (int i = 0; i < 200; i++){
   if(error_state || !stp_state){
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
void dispense(void){
  unsigned long last_incr = 0;
  float currSpeed = value.safe_speed;
  if(stp_state){
   digitalWrite(ena_Pin, HIGH);
   delayMicroseconds(100);
   
   // microseconds/step = mL/step * min/mL * 60000000 microseconds/min

   value.speed = (( (value.vol_per_1600steps / 1600) / value.vol_per_min) * 30000000);
   //Serial.print("Speed : "); 
   //Serial.println(speed); 
   if (value.volume > maxAllowed.volume){

   } else if (value.volume > 0) {
    
    digitalWrite(dir_Pin, LOW);
    delayMicroseconds(100);

    value.steps = value.volume / (value.vol_per_1600steps/1600);

     for (int i = 0; i < value.steps; i++){
      if(error_state || !stp_state){
       sendCommand("page 0");
       return;
      }
      //nexLoop(nex_listen_list);
      if((millis() - last_incr > value.accel_time) && currSpeed > value.speed_pull){
        currSpeed = currSpeed - value.accel_pull;
        last_incr = millis();
      }
      step(currSpeed);
     }

     digitalWrite(dir_Pin, HIGH);
     delayMicroseconds(value.sdelay);
     //Serial.println(sdelay);

     for (int i = 0; i < value.steps; i++){
      if(error_state || !stp_state){
       sendCommand("page 0");
       return;
      }
      //nexLoop(nex_listen_list);
      if((millis() - last_incr > value.accel_time) && currSpeed > value.speed_push){
        currSpeed = currSpeed - value.accel_push;
        last_incr = millis();
      }
      step(value.speed);
     }
   }
  }
  value.dispensed = false;
sendCommand("page 0");
}
void cal1600(void){
 //Serial.println("Dispensing cal1600");
 if(stp_state){
  digitalWrite(ena_Pin, HIGH);
  delayMicroseconds(100);

  digitalWrite(dir_Pin, LOW);
  delayMicroseconds(100);

  for (int i = 0; i < 1600; i++){
   if(error_state || !stp_state){
    sendCommand("page 0");
       return;
      }
   //nexLoop(nex_listen_list);
   step(value.safe_speed);
  }

  digitalWrite(dir_Pin, HIGH);
  delayMicroseconds(value.sdelay);

  for (int i = 0; i < 1600; i++){
   if(error_state || !stp_state){
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
void min_Limit(void){
  if(millis()-last_interrupt > 5000){
    minLimit_trigger = true;
    if(!zero_block){
     error_state = true;
    }
  }
  last_interrupt = millis();
   
}
void max_Limit(void){
  if(millis()-last_interrupt > 5000){
    maxLimit_trigger = true;
    if(!max_block){
     error_state = true;
    }
   }
  last_interrupt = millis();
}
void e_Stop(void){
  if(millis()-last_interrupt > 5000){
    eStop_trigger = true;
    error_state = true;
   }
  last_interrupt = millis();
}
void rstMCU(void){
 NVIC_SystemReset();
}
void safetyEna(void){
 attachInterrupt(digitalPinToInterrupt(min_Limit_sw_Pin), min_Limit, RISING);
 attachInterrupt(digitalPinToInterrupt(max_Limit_sw_Pin), max_Limit, RISING);
 attachInterrupt(digitalPinToInterrupt(estop_sw_Pin), e_Stop, RISING);
 //Serial.println("interrupts enabled");
}
void safetyDis(void){
 detachInterrupt(digitalPinToInterrupt(min_Limit_sw_Pin));
 detachInterrupt(digitalPinToInterrupt(max_Limit_sw_Pin));
 detachInterrupt(digitalPinToInterrupt(estop_sw_Pin));
 //Serial.println("interrupts disabled");
}
void printSettings(void){
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