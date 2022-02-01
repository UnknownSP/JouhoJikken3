#define MotorR_In1 2
#define MotorR_In2 3
#define MotorR_PWM 9
#define MotorL_In1 4
#define MotorL_In2 5
#define MotorL_PWM 10
#define LED 13

#define UL_R_TRIG 6
#define UL_R_ECHO 7
#define UL_L_TRIG 11
#define UL_L_ECHO 12

#define INTERVAL_MS 10

#define MAX_RANGE 250 //[mm]
#define MIN_RANGE 68 //[mm]

#define DUTY_OFFSET 25
#define MOTOR_R_OFFSET 0
#define MOTOR_L_OFFSET 0
#define MAX_STRAIGHT_OFFSET 35
#define MAX_CURVE_OFFSET 23

int Sys_counter_recent = 0;

int UL_R_distance = 0;
int UL_L_distance = 0;

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

typedef enum ul_sensor{
  UL_R = 0,
  UL_L = 1,
} UL_SENSOR;

void setup() {
  Serial.begin(9600);
  GPIO_init();
}

bool adjust_flag = false;
int start_time;
int stop_time;

void loop() {
  int motor_r_duty = 0;
  int motor_l_duty = 0;
  int straight_duty_offset = 0;
  int curve_duty_offset = 0;
  
  UL_R_distance = get_distance(UL_R, 0.0, false, false);
  UL_L_distance = get_distance(UL_L, 0.0, false, false);

  if(adjust_flag){
    if(millis()-start_time >= 1800){
      adjust_flag = false;
    }
    UL_L_distance -= (int)(50.0 * (1800.0 - (millis()-(double)start_time))/(1800.0));
    //UL_R_distance += 4;
    if(UL_L_distance <= MIN_RANGE){
      UL_L_distance = MIN_RANGE;
    }
  }

  straight_duty_offset = (int) ( (double)(min(UL_R_distance,UL_L_distance) - MIN_RANGE) / (double)(MAX_RANGE - MIN_RANGE) * (double)MAX_STRAIGHT_OFFSET ) ;
  curve_duty_offset = (int) ( (double)(UL_R_distance-UL_L_distance) / (double)(MAX_RANGE-MIN_RANGE) * (double)MAX_CURVE_OFFSET );

  if(min(UL_R_distance,UL_L_distance) == MIN_RANGE){
    motor_r_duty = 0;
    motor_l_duty = 0;
    if(millis()-stop_time >= 2000){
      adjust_flag = true;
      start_time = millis();
    }
  }else{
    stop_time = millis();
    if(adjust_flag){
      motor_r_duty = 4 + DUTY_OFFSET + MOTOR_R_OFFSET + straight_duty_offset + curve_duty_offset;
    }else{
      motor_r_duty = DUTY_OFFSET + MOTOR_R_OFFSET + straight_duty_offset + curve_duty_offset;
    }
    motor_r_duty = DUTY_OFFSET + MOTOR_R_OFFSET + straight_duty_offset + curve_duty_offset;
    motor_l_duty = DUTY_OFFSET + MOTOR_L_OFFSET + straight_duty_offset - curve_duty_offset;
  }
  digitalWrite(LED,adjust_flag);
  
  Ctrl_Motor(M_R,MS_FORWARD,motor_r_duty);
  Ctrl_Motor(M_L,MS_BACKWARD,motor_l_duty);

  //タイミング待ち
  while(true){
    if(millis()-Sys_counter_recent > INTERVAL_MS){
      Sys_counter_recent = millis();
      break;
    }
  }
}

int get_distance(UL_SENSOR side, float temp, bool en_temp, bool debug){
  float raw_dist = 0.0;
  static int recent_dist[2] = {0};
  int dist = 0;
  switch(side){
    case UL_R:
      digitalWrite(UL_R_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(UL_R_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(UL_R_TRIG, LOW);
      raw_dist = pulseIn(UL_R_ECHO, HIGH, 50000) / 2.0;
      break;
    case UL_L:
      digitalWrite(UL_L_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(UL_L_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(UL_L_TRIG, LOW);
      raw_dist = pulseIn(UL_L_ECHO, HIGH, 50000) / 2.0;
      break;
  }
  if(en_temp){
    dist =  int(raw_dist * (331.5 + 0.6*temp) * 0.001); 
  }else{
    dist =  int(raw_dist * 340.0 * 0.001);
  }
  if(dist == 0){
    dist = recent_dist[side];
  }else{
    if(dist >= MAX_RANGE){
      dist = recent_dist[side];
    }else if(dist <= MIN_RANGE){
      dist = MIN_RANGE;
    }
    recent_dist[side] = dist; 
  }
  if(debug){
    switch(side){
      case UL_R:
        Serial.print("UL_R [mm] : ");
        Serial.println(dist);
        break;
      case UL_L:
        Serial.print("UL_L [mm] : ");
        Serial.println(dist);
        break;
    }
  }
  return dist;
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

void GPIO_init() {
  pinMode(MotorR_In1,OUTPUT);
  pinMode(MotorR_In2,OUTPUT);
  pinMode(MotorR_PWM,OUTPUT);
  pinMode(MotorL_In1,OUTPUT);
  pinMode(MotorL_In2,OUTPUT);
  pinMode(MotorL_PWM,OUTPUT);
  pinMode(UL_R_TRIG, OUTPUT);
  pinMode(UL_R_ECHO, INPUT);
  pinMode(UL_L_TRIG, OUTPUT);
  pinMode(UL_L_ECHO, INPUT);
  pinMode(LED, OUTPUT);
}
