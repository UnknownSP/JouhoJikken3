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

#define INTERVAL_MS 10

#define HIGH_THRESHOLD  800
#define LOW_THRESHOLD   100

#define SPEEDDOWN_MS 0
#define MotorDuty_Offset_adjust_max 0
#define PID_MaxDuty_adjust_max 0
#define PID_coeff_adjust_max 0.0

#define MOTORDUTY_OFFSET 26
#define PID_MAXDUTY 20
#define PID_COEFF 2.2
#define KP 0.0029
#define KI 0.0041
#define KD 0.0004
#define SPEEDUP_MS 1500


typedef enum operation_mode{
  OP_CURVE_FAST,
  OP_CURVE_SLOW,
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

ROBOT_STAT robot_position = RS_MID;
ROBOT_STAT robot_position_recent = RS_MID;
OP_MODE operation_mode = OP_CURVE_FAST;

long Sys_counter = 0; //[ms]
long Sys_counter_recent = 0;
long recent_cal_time = 0;
long recent_over_time = -SPEEDUP_MS;

bool _back_mode = true;
bool _mode_change = false;
bool _turn_mode = false;
bool _MotorCtrlItrpt = false;
bool _SpeedUp = true;
bool _SpeedDown = true;

int MotorDuty_Offset_adjust = 0;
int PID_MaxDuty_adjust = 0;
double PID_coeff_adjust = 0.0;

int Cal_PID(int now_diff, int recent_diff, int delta_t/*[ms]*/, double reset);

void setup() {
  GPIO_init();
  //Timer_init();
  Mode_init();
  
  _back_mode = false;
  _mode_change = true;

  Serial.begin(9600);
}


bool syscount_update = false;

void loop() {
  int sensor_val_FR = analogRead(Sensor_FR);
  int sensor_val_FL = analogRead(Sensor_FL);
  int sensor_val_FM = analogRead(Sensor_FM);
  int motor_r_duty = 0;
  int motor_l_duty = 0;
  static int now_FSdiff = 0;
  static int recent_FSdiff = 0;
  static int cal_pid;

  Cal_Sensor_stat(sensor_val_FR, sensor_val_FL, sensor_val_FM);
  
  Cal_aboutPosition_Front(false);

  now_FSdiff = sensor_val_FL - sensor_val_FR;

  if((robot_position_recent==RS_OVERRIGHT && robot_position==RS_RIGHT) || (robot_position_recent==RS_OVERLEFT && robot_position==RS_LEFT)){
    delay(50);
    Ctrl_Motor(M_R,MS_BRAKE,0);
    Ctrl_Motor(M_L,MS_BRAKE,0);
    Cal_aboutPosition_Front(false); 
    delay(50);
    Cal_aboutPosition_Front(false); 
  }
  
  
  cal_pid = Cal_PID(now_FSdiff,recent_FSdiff,millis()-recent_cal_time,false);
  if(operation_mode == OP_CURVE_FAST){
    motor_r_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) + cal_pid;
    motor_l_duty = (MOTORDUTY_OFFSET+MotorDuty_Offset_adjust) - cal_pid; 
  }else if(operation_mode == OP_CURVE_SLOW){
    motor_r_duty = (MOTORDUTY_OFFSET-2+MotorDuty_Offset_adjust) + cal_pid;
    motor_l_duty = (MOTORDUTY_OFFSET-2+MotorDuty_Offset_adjust) - cal_pid; 
  }
  
  recent_cal_time = millis();

  motor_r_duty = constrain(motor_r_duty, 0, 100);
  motor_l_duty = constrain(motor_l_duty, 0, 100);


  if(operation_mode == OP_CURVE_FAST || operation_mode == OP_CURVE_SLOW){
    switch(robot_position){
    case RS_MID:
    case RS_MIDRIGHT:
    case RS_MIDLEFT:
    case RS_RIGHT:
    case RS_LEFT:
      _MotorCtrlItrpt = false;
      break;
    case RS_OVERRIGHT:
      Ctrl_Motor(M_R,MS_FORWARD,MOTORDUTY_OFFSET);
      Ctrl_Motor(M_L,MS_BACKWARD,13); 
      _MotorCtrlItrpt = true;
      cal_pid = Cal_PID(0,0,0,true);
      //if(_SpeedUp) recent_over_time = millis();
      MotorDuty_Offset_adjust = 0;
      PID_MaxDuty_adjust = 0;
      PID_coeff_adjust = 0.0;
      //_SpeedUp = false;
      break;
    case RS_OVERLEFT:
      Ctrl_Motor(M_R,MS_BACKWARD,13);
      //Ctrl_Motor(M_R,MS_FORWARD,0);
      Ctrl_Motor(M_L,MS_FORWARD,MOTORDUTY_OFFSET); 
      _MotorCtrlItrpt = true;
      cal_pid = Cal_PID(0,0,0,true);
      //if(_SpeedUp) recent_over_time = millis();
      MotorDuty_Offset_adjust = 0;
      PID_MaxDuty_adjust = 0;
      PID_coeff_adjust = 0.0;
      //_SpeedUp = false;
      break;
    }
  }

  if(_SpeedUp){
    if(SPEEDUP_MS - millis() >= 0){
      double coeff = ((double)millis() / (double)SPEEDUP_MS);
      if(coeff >= 1.0){
        coeff = 1.0;
      }
      motor_r_duty = (int)((double)motor_r_duty * coeff);
      motor_l_duty = (int)((double)motor_l_duty * coeff);
    }else{
      _SpeedUp = false;
    }
  }
  
  if(!_MotorCtrlItrpt){
    Ctrl_Motor(M_R,MS_FORWARD,motor_r_duty);
    Ctrl_Motor(M_L,MS_FORWARD,motor_l_duty);  
  }

  if(_turn_mode && !_back_mode){
    _MotorCtrlItrpt = true;
    Ctrl_Motor(M_R,MS_BACKWARD,14);
    Ctrl_Motor(M_L,MS_FORWARD,17);
  }

  recent_FSdiff = now_FSdiff;
  FR_stat_recent = FR_stat;
  FL_stat_recent = FL_stat;
  FM_stat_recent = FM_stat;
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

void Cal_Sensor_stat(int fr, int fl, int fm){
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
        Serial.println("[]               |||||               []");
        break;
      case RS_MIDRIGHT:
        Serial.println("[]          |||||                    []");
        break;
      case RS_MIDLEFT:
        Serial.println("[]                    |||||          []");
        break;
      case RS_RIGHT:
        Serial.println("[]     |||||                         []");
        break;
      case RS_LEFT:
        Serial.println("[]                         |||||     []");
        break;
      case RS_OVERRIGHT:
        Serial.println("[]|||||                              []");
        break;
      case RS_OVERLEFT:
        Serial.println("[]                              |||||[]");
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
    //operation_mode = OP_STRAIGHT_BACK;
  }else if(digitalRead(ModePin_2) == LOW){
    operation_mode = OP_CURVE_SLOW;
  }else if(digitalRead(ModePin_3) == LOW){
    operation_mode = OP_CURVE_FAST;
  }else{
    operation_mode = OP_CURVE_FAST;
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
