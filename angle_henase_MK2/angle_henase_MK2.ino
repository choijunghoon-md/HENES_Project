#include <MsTimer2.h>
#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23
#define alpha 0.8
#define AD_MAX 900
#define AD_MIN 100
#define MOTOR2_PWM 8
#define MOTOR2_ENA 9
#define MOTOR2_ENB 10
#define NEURAL_ANGLE 3
#define LEFT_STEER_ANGLE  -30
#define RIGHT_STEER_ANGLE  30
#define No_Calibriton_Point 17
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4
#define MOTOR3_PWM 5
#define MOTOR3_ENA 6
#define MOTOR3_ENB 7
#define M_1_PULSE 344
#define M_2_PULSE 343
#define PULSE_1_M 1./344.
#define PULSE_2_M 1./343.
#define VEL_1_PULSE 6.88
#define VEL_2_PULSE 6.86

// Front Motor Drive Pin 설정

float encoder1_taget_speed=5.0;
float encoder2_taget_speed=5.0;

int f_speed = 0, r_speed = 0;  // 속도

struct {
    double X[No_Calibriton_Point]; //x값
    double Y[No_Calibriton_Point]; //y값
}cal_data;

double Setpoint, Input, Output;
float pwm_output=0.0;
double error, error_old;
double error_s, error_d;

float Kp = 0.55*1.2;
float Ki = 0.1*1.4;
float Kd = 3*1.2;

int Steer_Angle_Measure = 0;
int Steering_Angle = NEURAL_ANGLE;

float avg_old = 0.0;
float avg = 0;
int ad_value = 0;

int steer_angle = 0;

signed long encoder1count = 0;
signed long encoder2count = 0;

signed long encoder1_error = 0;
signed long encoder2_error = 0;

signed long encoder1_error_d = 0;
signed long encoder2_error_d = 0;

signed long encoder1_target = 0;
signed long encoder2_target = 0;

signed long encoder1_error_old = 0; 
signed long encoder2_error_old = 0;

signed long encoder1_error_sum = 0; 
signed long encoder2_error_sum = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);
  // Rear Motor Drive Pin Setup
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);
  initEncoders();          // initialize encoder
  clearEncoderCount(1); 
  clearEncoderCount(2); 
  pinMode(MOTOR2_PWM,OUTPUT);
  pinMode(MOTOR2_ENA,OUTPUT);
  pinMode(MOTOR2_ENB,OUTPUT);
  MsTimer2::set(5, control_callback);
  MsTimer2::start();
  cal_data = {
    {100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900},
    {14.35,13.3,11.98,10.43,8.25,6.65,4.77,3.32,-0.085,-2.41,-4.76,-7,-9.85,-12.14,-14.27,-16.77,-18.99}
    };
}

int flag=1;

void loop() {
  // put your main code here, to run repeatedly:
  float target_distance=0.3;
  
  //encoder1_target = target_distance*M_1_PULSE;
  //encoder2_target = target_distance*M_2_PULSE;
  
  Steering_Angle=500;
  PID_Control();

  Serial.print("front_speed : "); Serial.print(encoder1_taget_speed); Serial.print("rear_speed : "); Serial.println(encoder2_taget_speed); 
}

void steer_motor_control(int motor_pwm)
{
  if((avg <= AD_MIN) || (avg <= AD_MIN) )
  {
     digitalWrite(MOTOR2_ENA, LOW);
     digitalWrite(MOTOR2_ENB, LOW);
     analogWrite(MOTOR2_PWM, 0);
  }

  
  else if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, 0);
  }

  
}

void PID_Control()
{
  error = Steering_Angle - avg ;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  55) ?  55 : pwm_output;
  pwm_output = (pwm_output <= -55) ? -55 : pwm_output;

  if (fabs(error) <= 0.2)
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else          steer_motor_control(pwm_output);
  error_old = error;  
}


void control_callback()
{  
  // read the analog in value:
  ad_value = analogRead(A15);
  avg = avg_old*alpha + (1.0-alpha)*ad_value;
  avg_old = avg;

  encoder1_target += encoder1_taget_speed*VEL_1_PULSE;
  encoder2_target += encoder1_taget_speed*VEL_2_PULSE;
  
  static boolean output = HIGH;
  digitalWrite(13, output);
  
  output = !output;
  
  front_motor_PID_control();
  rear_motor_PID_control();
}

double linear_mapping(double x) {
    double y;
    for (int i = 0; i < No_Calibriton_Point; i++) 
    {
        if (x <= cal_data.X[0]) //x값이 범위 아래로 있을때
        {
            y = ((cal_data.Y[0] - cal_data.Y[1]) / (cal_data.X[0] - cal_data.X[1])) * (x - cal_data.X[1]) + cal_data.Y[1]; //처음으로 생성된 선형함수 이용
            break;
        }
        else if (x > cal_data.X[i] && x <= cal_data.X[i+1]) //x값이 범위안에 있을때
        {
            y = ((cal_data.Y[i] - cal_data.Y[i + 1]) / (cal_data.X[i] - cal_data.X[i + 1])) * (x - cal_data.X[i + 1]) + cal_data.Y[i + 1]; //범위 안에 있는 선형함수를 이용
            break;
        }
        else if (x >= cal_data.X[No_Calibriton_Point - 1]) //값이 범위 위에 있을때
        {
            y = ((cal_data.Y[No_Calibriton_Point - 1] - cal_data.Y[No_Calibriton_Point - 2]) / (cal_data.X[No_Calibriton_Point - 1] - cal_data.X[No_Calibriton_Point - 2])) * (x - cal_data.X[No_Calibriton_Point - 2]) + cal_data.Y[No_Calibriton_Point - 2]; //마지막으로 생성된 선형함수 이용
            break;
        }
    }
    return y;
}

void front_motor_control(int motor1_pwm)
{
   if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);   // 방향은 설정에 따라 바꿀 것
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor3_pwm)
{
   if (motor3_pwm < 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, -motor3_pwm);
  }

  else if (motor3_pwm > 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, motor3_pwm);
  }
  else
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    digitalWrite(MOTOR3_PWM, 0);
  }
}

void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}
void initEncoders()   //encoder 초기화
{
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);

  SPI.begin();

  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  return -count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}

float Kp_motor = 0.3*0.691-0.0001;
float Kd_motor = 0.5*1.8;
float Ki_motor = 0;
float front_motor_pwm=0.0;

void front_motor_PID_control(void)
{
  encoder1count = readEncoder(1);
  encoder1_error = encoder1_target - encoder1count;
  //encoder1_error_sum += encoder1_error;

  encoder1_error_d = encoder1_error - encoder1_error_old;
  //encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
  //encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;

  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >=  55) ?  55 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -55) ? -55 : front_motor_pwm;

 if (fabs(encoder1_error) <= 2)
  {
    encoder1_error_sum = 0;
  }

  else 
  {
    front_motor_control(front_motor_pwm);    
  }
  encoder1_error_old = encoder1_error;
}

float Kp2_motor = 0.3*0.566-0.0001;
float Kd2_motor = 0.5*1.8;
float Ki2_motor = 0;
float rear_motor_pwm=0.0;

void rear_motor_PID_control(void)
{
  encoder2count = readEncoder(2);
  encoder2_error = encoder2_target - encoder2count;
 // encoder2_error_sum += encoder2_error;

  encoder2_error_d = encoder2_error - encoder2_error_old;
 // encoder2_error_sum = (encoder2_error_sum >=  100) ?  100 : encoder2_error_sum;
  //encoder2_error_sum = (encoder2_error_sum <= -100) ? -100 : encoder2_error_sum;

  rear_motor_pwm = Kp2_motor * encoder2_error + Kd2_motor * encoder2_error_d + Ki2_motor * encoder2_error_sum;
  rear_motor_pwm = (rear_motor_pwm >=  155) ?  155 : rear_motor_pwm;
  rear_motor_pwm = (rear_motor_pwm <= -155) ? -155 : rear_motor_pwm;

 if (fabs(encoder2_error) <= 2)
  {
    encoder2_error_sum = 0;
  }

  else 
  {
    rear_motor_control(rear_motor_pwm);    
  }
  encoder2_error_old = encoder2_error;
}
