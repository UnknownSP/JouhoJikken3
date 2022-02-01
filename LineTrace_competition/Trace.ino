#define MotorR_In1 2
#define MotorR_In2 3
#define MotorR_PWM 9
#define MotorL_In1 4
#define MotorL_In2 5
#define MotorL_PWM 10

#define ModePin_1 11
#define ModePin_2 12
#define ModePin_3 13

#define Sensor_FR A0
#define Sensor_FL A1
#define Sensor_FM A5
#define Sensor_BR A2
#define Sensor_BL A3
#define Sensor_BM A4

#define INTERVAL_MS 10

#define HIGH_THRESHOLD  800
#define LOW_THRESHOLD   100

#define SPEEDUP_MS 2700
#define MotorDuty_Offset_adjust_max 6
#define PID_MaxDuty_adjust_max 5
#define PID_coeff_adjust_max 0.55

#define MOTORDUTY_OFFSET 17
#define PID_MAXDUTY 20
#define PID_COEFF 1.8
#define KP 0.0032
#define KI 0.0065
#define KD 0.0004

typedef enum operation_mode{
  OP_STRAIGHT_BACK,
  OP_STRAIGHT_TURN,
  OP_CURVE,
} OP_MODE;

typedef enum sensor_status{
  ON_WHITE,
  ON_BLACK,
  ON_MID,
} SENSOR_STAT;

typedef enum robot_status{
  RS_MID = 0,
  RS_MIDRIGHT = 1,
  RS_MIDLEFT = -1,
  RS_RIGHT = 2,
  RS_LEFT = -2,
  RS_OVERRIGHT = 3,
  RS_OVERLEFT = -3,
} ROBOT_STAT;

typedef enum motor_status{
  MS_FREE,
  MS_FORWARD,
  MS_BACKWARD,
  MS_BRAKE,
} MOTOR_STAT;

typedef enum motor{
  M_R,
  M_L,
} MOTOR;

SENSOR_STAT FR_stat = ON_MID;
SENSOR_STAT FL_stat = ON_MID;
SENSOR_STAT FM_stat = ON_MID;
SENSOR_STAT FR_stat_recent = ON_MID;
SENSOR_STAT FL_stat_recent = ON_MID;
SENSOR_STAT FM_stat_recent = ON_MID;
SENSOR_STAT BR_stat = ON_MID;
SENSOR_STAT BL_stat = ON_MID;
SENSOR_STAT BM_stat = ON_MID;
SENSOR_STAT BR_stat_recent = ON_MID;
SENSOR_STAT BL_stat_recent = ON_MID;
SENSOR_STAT BM_stat_recent = ON_MID;

ROBOT_STAT robot_position = RS_MID;
ROBOT_STAT robot_position_recent = RS_MID;
OP_MODE operation_mode = OP_CURVE;

long Sys_counter = 0; //[ms]
long Sys_counter_recent = 0;
long recent_cal_time = 0;
long recent_over_time = 0;

bool _back_mode = true;
bool _mode_change = false;
bool _turn_mode = false;
bool _MotorCtrlItrpt = false;

int MotorDuty_Offset_adjust = 0;
int PID_MaxDuty_adjust = 0;
double PID_coeff_adjust = 0.0;

int Cal_PID(int now_diff, int recent_diff, int delta_t/*[ms]*/, double reset);

void setup() {
  GPIO_init();
  //Timer_init();
  Mode_init();
  
  if(operation_mode == OP_CURVE){
    _back_mode = false;
    _mode_change = true;
  }else if(operation_mode == OP_STRAIGHT_TURN){
    _back_mode = true;
    _mode_change = false;
    _turn_mode = true;
  }

  Serial.begin(9600);
}


bool syscount_update = false;

void loop() {
  int sensor_val_FR = analogRead(Sensor_FR);
  int sensor_val_FL = analogRead(Sensor_FL);
  int sensor_val_FM = analogRead(Sensor_FM);
  int sensor_val_BR = analogRead(Sensor_BR);
  int sensor_val_BL = analogRead(Sensor_BL);
  int sensor_val_BM = analogRead(Sensor_BM);
  int motor_r_duty = 0;
  int motor_l_duty = 0;
  static int now_FSdiff = 0;
  static int recent_FSdiff = 0;
  static int now_BSdiff = 0;
  static int recent_BSdiff = 0;
  static int cal_pid;

  Cal_Sensor_stat(sensor_val_FR, sensor_val_FL, sensor_val_FM, sensor_val_BR, sensor_val_BL, sensor_val_BM);
  
  if(_back_mode){
    Cal_aboutPosition_Back(false);
  }else{
    Cal_aboutPosition_Front(false); 
  }

  now_FSdiff = sensor_val_FL - sensor_val_FR;
  now_BSdiff = sensor_val_BR - sensor_val_BL;

  if((robot_position_recent==RS_OVERRIGHT && robot_position==RS_RIGHT) || (robot_position_recent==RS_OVERLEFT && robot_position==RS_LEFT)){
    delay(50);
    Ctrl_Motor(M_R,MS_BRAKE,0);
    Ctrl_Motor(M_L,MS_BRAKE,0);
    if(_back_mode){
      Cal_aboutPosition_Back(false);
    }else{
      Cal_aboutPosition_Front(false); 
    }
    delay(50);
    if(_back_mode){
      Cal_aboutPosition_Back(false);
    }else{
      Cal_aboutPosition_Front(false); 
    }
  }
  
  if(_back_mode){
    cal_pid = Cal_PID(now_BSdiff,recent_BSdiff,millis()-recent_cal_time,false);
    motor_r_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) - cal_pid;
    motor_l_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) + cal_pid; 
  }else{
    cal_pid = Cal_PID(now_FSdiff,recent_FSdiff,millis()-recent_cal_time,false);
    motor_r_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) + cal_pid;
    motor_l_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) - cal_pid; 
  } 
  recent_cal_time = millis();

  motor_r_duty = constrain(motor_r_duty, 0, 100);
  motor_l_duty = constrain(motor_l_duty, 0, 100);

  if(operation_mode == OP_CURVE){
    switch(robot_position){
    case RS_MID:
    case RS_MIDRIGHT:
    case RS_MIDLEFT:
    case RS_RIGHT:
    case RS_LEFT:
      if(millis()-recent_over_time >= SPEEDUP_MS){
        MotorDuty_Offset_adjust = (int)((double)(millis()-recent_over_time+SPEEDUP_MS)*0.004);
        PID_MaxDuty_adjust = (int)((double)(millis()-recent_over_time+SPEEDUP_MS)*0.004);
        PID_coeff_adjust = (double)(millis()-recent_over_time+SPEEDUP_MS)*0.0004;
        if(MotorDuty_Offset_adjust >= MotorDuty_Offset_adjust_max) MotorDuty_Offset_adjust = MotorDuty_Offset_adjust_max;
        if(PID_MaxDuty_adjust >= PID_MaxDuty_adjust_max) PID_MaxDuty_adjust = PID_MaxDuty_adjust_max;
        if(PID_coeff_adjust >= PID_coeff_adjust_max) PID_coeff_adjust = PID_coeff_adjust_max;
      }else{
        MotorDuty_Offset_adjust = 0;
        PID_MaxDuty_adjust = 0;
        PID_coeff_adjust = 0.0;
      }
      _MotorCtrlItrpt = false;
      break;
    case RS_OVERRIGHT:
      if(_back_mode){
        Ctrl_Motor(M_R,MS_FORWARD,13);
        Ctrl_Motor(M_L,MS_BACKWARD,MOTORDUTY_OFFSET);
      }else{
        Ctrl_Motor(M_R,MS_FORWARD,MOTORDUTY_OFFSET);
        Ctrl_Motor(M_L,MS_BACKWARD,13); 
      } 
      _MotorCtrlItrpt = true;
      cal_pid = Cal_PID(0,0,0,true);
      recent_over_time = millis();
      MotorDuty_Offset_adjust = 0;
      PID_MaxDuty_adjust = 0;
      PID_coeff_adjust = 0.0;
      break;
    case RS_OVERLEFT:
      if(_back_mode){
        Ctrl_Motor(M_R,MS_BACKWARD,MOTORDUTY_OFFSET);
        Ctrl_Motor(M_L,MS_FORWARD,13);
      }else{
        Ctrl_Motor(M_R,MS_BACKWARD,13);
        Ctrl_Motor(M_L,MS_FORWARD,MOTORDUTY_OFFSET); 
      } 
      _MotorCtrlItrpt = true;
      cal_pid = Cal_PID(0,0,0,true);
      recent_over_time = millis();
      MotorDuty_Offset_adjust = 0;
      PID_MaxDuty_adjust = 0;
      PID_coeff_adjust = 0.0;
      break;
    }
  }
  
  if(!_MotorCtrlItrpt){
    if(_back_mode){
      Ctrl_Motor(M_R,MS_BACKWARD,motor_r_duty);
      Ctrl_Motor(M_L,MS_BACKWARD,motor_l_duty);
    }else{
      Ctrl_Motor(M_R,MS_FORWARD,motor_r_duty);
      Ctrl_Motor(M_L,MS_FORWARD,motor_l_duty); 
    } 
  }

  if(BM_stat == ON_WHITE && !_mode_change){
    _back_mode = false;
    _mode_change = true;
    cal_pid = Cal_PID(0,0,0,true);
    delay(100);
  }
  if(_turn_mode && !_back_mode){
    _MotorCtrlItrpt = true;
    Ctrl_Motor(M_R,MS_BACKWARD,14);
    Ctrl_Motor(M_L,MS_FORWARD,17);
    if((BL_stat == ON_BLACK || BL_stat == ON_MID) && (BM_stat == ON_BLACK || BM_stat==ON_MID)){
      cal_pid = Cal_PID(0,0,0,true);
      Ctrl_Motor(M_R,MS_BRAKE,0);
      Ctrl_Motor(M_L,MS_BRAKE,0);
      _turn_mode = false;
      _back_mode = true;
      _MotorCtrlItrpt = false;
      delay(100);
    }
  }

  recent_FSdiff = now_FSdiff;
  recent_BSdiff = now_BSdiff;
  FR_stat_recent = FR_stat;
  FL_stat_recent = FL_stat;
  FM_stat_recent = FM_stat;
  BR_stat_recent = BR_stat;
  BL_stat_recent = BL_stat;
  BM_stat_recent = BM_stat;
  robot_position_recent = robot_position;

  /*タイミング待ち*/
  while(true){
    if(millis()-Sys_counter_recent > INTERVAL_MS){
      //Serial.println(millis()-Sys_counter_recent);
      Sys_counter_recent = millis();
      break;
    }
  }
}

/*------------------------------------------------------*/

/*
ISR (TIMER2_COMPA_vect) {
  //Sys_counter += 1;
  if(syscount_update){
    Sys_counter_recent = Sys_counter;
    syscount_update = false;
  }
}*/

int Cal_PID(int now_diff, int recent_diff, int delta_t/*[ms]*/, double reset){
  double p,i,d;
  static double integral = 0.0;
  static int return_pid = 0;
  if(reset){
    integral = 0.0;
    return_pid = 0;
    return 0;
  }

  if(abs(return_pid) <= (PID_MAXDUTY+PID_MaxDuty_adjust)){
    integral += ((double)now_diff+(double)recent_diff)/2.0 * ((double)delta_t/1000.0); 
  }
  p = KP * (double)now_diff;
  i = KI * integral;
  d = KD * ((double)now_diff-(double)recent_diff) / ((double)delta_t/1000.0);

  return_pid = (int)((p+i+d)*(PID_COEFF+PID_coeff_adjust));
  
  return constrain(return_pid,-(PID_MAXDUTY+PID_MaxDuty_adjust),(PID_MAXDUTY+PID_MaxDuty_adjust));
}

void Ctrl_Motor(MOTOR side, MOTOR_STAT stat, int duty){
  bool in_1 = LOW;
  bool in_2 = LOW;
  switch(stat){
    case MS_FREE:
      in_1 = LOW;
      in_2 = LOW;
      break;
    case MS_FORWARD:
      in_1 = LOW;
      in_2 = HIGH;
      break;
    case MS_BACKWARD:
      in_1 = HIGH;
      in_2 = LOW;
      break;
    case MS_BRAKE:
      in_1 = HIGH;
      in_2 = HIGH;
      break;
  }
  switch(side){
    case M_R:
      digitalWrite(MotorR_In1,in_1);
      digitalWrite(MotorR_In2,in_2);
      analogWrite(MotorR_PWM,map(duty, 0, 100, 0, 255));
      break;
    case M_L:
      digitalWrite(MotorL_In1,in_1);
      digitalWrite(MotorL_In2,in_2);
      analogWrite(MotorL_PWM,map(duty, 0, 100, 0, 255));
      break;
  }
}

void Cal_Sensor_stat(int fr, int fl, int fm, int br, int bl, int bm){
  if(fr >= HIGH_THRESHOLD){
    FR_stat = ON_BLACK;
  }else if(fr <= LOW_THRESHOLD){
    FR_stat = ON_WHITE;
  }else{
    FR_stat = ON_MID;  
  }
  if(fl >= HIGH_THRESHOLD){
    FL_stat = ON_BLACK;
  }else if(fl <= LOW_THRESHOLD){
    FL_stat = ON_WHITE;
  }else{
    FL_stat = ON_MID;  
  }
  if(fm >= HIGH_THRESHOLD){
    FM_stat = ON_BLACK;
  }else if(fm <= LOW_THRESHOLD){
    FM_stat = ON_WHITE;
  }else{
    FM_stat = ON_MID;  
  }
  if(br >= HIGH_THRESHOLD){
    BR_stat = ON_BLACK;
  }else if(br <= LOW_THRESHOLD){
    BR_stat = ON_WHITE;
  }else{
    BR_stat = ON_MID;  
  }
  if(bl >= HIGH_THRESHOLD){
    BL_stat = ON_BLACK;
  }else if(bl <= LOW_THRESHOLD){
    BL_stat = ON_WHITE;
  }else{
    BL_stat = ON_MID;  
  }
  if(bm >= HIGH_THRESHOLD){
    BM_stat = ON_BLACK;
  }else if(bm <= LOW_THRESHOLD){
    BM_stat = ON_WHITE;
  }else{
    BM_stat = ON_MID;  
  }
}

void Cal_aboutPosition_Front(bool debug){

  if(robot_position_recent == RS_OVERRIGHT){
    if(FL_stat==ON_MID || FL_stat==ON_BLACK) robot_position = RS_RIGHT;
  }else if(robot_position_recent == RS_OVERLEFT){
    if(FR_stat==ON_MID || FR_stat==ON_BLACK) robot_position = RS_LEFT;
  }else if(robot_position_recent==RS_LEFT && FR_stat==ON_WHITE){
    robot_position = RS_OVERLEFT;
  }else if(robot_position_recent==RS_RIGHT && FL_stat==ON_WHITE){
    robot_position = RS_OVERRIGHT;
  }else if(robot_position_recent==RS_LEFT && FM_stat==ON_BLACK){
    robot_position = RS_MIDLEFT;
  }else if(robot_position_recent==RS_RIGHT && FM_stat==ON_BLACK){
    robot_position = RS_MIDRIGHT;
  }else if(robot_position_recent==RS_MIDLEFT && FM_stat==ON_WHITE){
    robot_position = RS_LEFT;
  }else if(robot_position_recent==RS_MIDRIGHT && FM_stat==ON_WHITE){
    robot_position = RS_RIGHT;
  }else if(FL_stat==ON_WHITE && FM_stat==ON_BLACK && FR_stat==ON_WHITE){
    robot_position = RS_MID;
  }else if(robot_position_recent!=RS_LEFT && FL_stat==ON_WHITE && (FR_stat==ON_MID || FR_stat==ON_BLACK)){
    robot_position = RS_MIDLEFT;
  }else if(robot_position_recent!=RS_RIGHT && (FL_stat==ON_MID || FL_stat==ON_BLACK) && FR_stat==ON_WHITE){
    robot_position = RS_MIDRIGHT;
  }

  if(debug){
    switch(robot_position){
      case RS_MID:
        Serial.println("[]          |||          []");
        break;
      case RS_MIDRIGHT:
        Serial.println("[]      |||              []");
        break;
      case RS_MIDLEFT:
        Serial.println("[]             |||       []");
        break;
      case RS_RIGHT:
        Serial.println("[]   |||                 []");
        break;
      case RS_LEFT:
        Serial.println("[]                 |||   []");
        break;
      case RS_OVERRIGHT:
        Serial.println("[]|||                    []");
        break;
      case RS_OVERLEFT:
        Serial.println("[]                    |||[]");
        break;
    }
  }
}

void Cal_aboutPosition_Back(bool debug){

  if(robot_position_recent == RS_OVERRIGHT){
    if(BM_stat==ON_MID) robot_position = RS_RIGHT;
  }else if(robot_position_recent == RS_OVERLEFT){
    if(BM_stat==ON_MID) robot_position = RS_LEFT;
  }else if(robot_position_recent==RS_LEFT && BM_stat==ON_WHITE){
    robot_position = RS_OVERLEFT;
  }else if(robot_position_recent==RS_RIGHT && BM_stat==ON_WHITE){
    robot_position = RS_OVERRIGHT;
  }else if(BR_stat==ON_WHITE && BM_stat==ON_BLACK && BL_stat==ON_WHITE){
    robot_position = RS_MID;
  }else if(BR_stat==ON_WHITE && (BL_stat==ON_MID || BL_stat==ON_BLACK)){
    robot_position = RS_LEFT;
  }else if((BR_stat==ON_MID || BR_stat==ON_BLACK) && BL_stat==ON_WHITE){
    robot_position = RS_RIGHT;
  }

  if(debug){
    switch(robot_position){
      case RS_MID:
        Serial.println("[]       |||       []");
        break;
      case RS_RIGHT:
        Serial.println("[]    |||          []");
        break;
      case RS_LEFT:
        Serial.println("[]          |||    []");
        break;
      case RS_OVERRIGHT:
        Serial.println("[]|||              []");
        break;
      case RS_OVERLEFT:
        Serial.println("[]              |||[]");
        break;
    }
  }
}

void Timer_init() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= 0b00000010; //CTC Mode
  TCCR2B |= 0b00000101; //1/128
  OCR2A  = 125-1;
  TIMSK2 |= (1 << OCIE2A);

  // 1/(16M * (1/128)) * 125 = 1m
  // 1ms timer
}

void Mode_init(){
  if(digitalRead(ModePin_1) == LOW){
    operation_mode = OP_STRAIGHT_BACK;
  }else if(digitalRead(ModePin_2) == LOW){
    operation_mode = OP_STRAIGHT_TURN;
  }else if(digitalRead(ModePin_3) == LOW){
    operation_mode = OP_CURVE;
  }else{
    operation_mode = OP_CURVE;
  }
}

void GPIO_init() {
  pinMode(MotorR_In1,OUTPUT);
  pinMode(MotorR_In2,OUTPUT);
  pinMode(MotorR_PWM,OUTPUT);
  pinMode(MotorL_In1,OUTPUT);
  pinMode(MotorL_In2,OUTPUT);
  pinMode(MotorL_PWM,OUTPUT);
  pinMode(ModePin_1, INPUT_PULLUP);
  pinMode(ModePin_2, INPUT_PULLUP);
  pinMode(ModePin_3, INPUT_PULLUP);
}
