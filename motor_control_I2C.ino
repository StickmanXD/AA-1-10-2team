
///////////////////// Steering Servo Control /////////////////////
#define RC_SERVO_PIN 8					// 서보 모터 핀 번호
#define NEURAL_ANGLE 90					// 조향을 위한 중립 각도 -> 나중에 변경
#define LEFT_STEER_ANGLE  -40				// 조향을 최대로 했을때 값. -> 너무 작으면 회전이 충분하지 못함. -> 나중에 변경
#define RIGHT_STEER_ANGLE  40				// 조향을 최대로 했을때 값. _> 너무 작으면 회전이 충분하지 못함. -> 나중에 변경

#include <Servo.h>						// 아두이노 서보 클래스 라이브러리
Servo   Steeringservo;					// Servo 라는 클래스를 사용하기 위함, Steeringservo 클래스를 사용할 수 있음.
int Steering_Angle = NEURAL_ANGLE;

void steering_control()					// 위 if = 왼쪽으로 최솟값, 밑 if = 오른쪽으로 최솟값.
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE; 	// 최대 50도
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;	// 최대 130도
  Steeringservo.write(Steering_Angle);  
}

///////////////////// Steering Servo Control /////////////////////


/////////////////////// DC Motor Control /////////////////////

#define MOTOR_PWM 5 
#define MOTOR_DIR 4
int Motor_Speed =0;

void motor_control(int dir, int motor_pwm)
{
  
  if(dir == 1) // forward
  { 
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM,motor_pwm);
    
  }
  else if(dir == -1) // backward
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM,motor_pwm);
  }
  else // stop
  {
    analogWrite(MOTOR_PWM,0);
  }
  
  
}

/////////////////////// DC Motor Control /////////////////////

///////////////////// I2C 통신 /////////////////////
#include <Wire.h>

void receiveEvent(int howMany)
{
  uint8_t a[4];;
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  Serial.print(a[0]); Serial.print(a[1]); Serial.print(a[2]); Serial.println(a[3]);
  Steering_Angle = a[1];
  Motor_Speed    = a[3]-128;
}
void requestEvent() {
  int8_t s;
  s= 100;
  Wire.write(s); // respond 
}
///////////////////// I2C 통신 /////////////////////

//////////////////////// Serial Comm.  //////////////////////////
/*
static uint8_t no_data = 0;        // number of data  from serial comm.
static uint8_t data[20] = {0, };   //  data array

void read_serial_com(void)
{
  if(Serial.available() > 0)
  {

    uint8_t temp = Serial.read();  // 1 byte read 
    data[no_data] = temp;
    no_data++;  
    if(  temp == '\n')
    {
      if(no_data>=5)
      { 
         if(data[no_data-5] == 'S')
        {
            Steering_Angle = data[no_data-4];
        }
        if(data[no_data-3] == 'D')
        {
           Motor_Speed    = data[3]-128;
        }
        
      }
      no_data = 0;
      memset(data,0,20);
      
    }
    
  } 
}

//////////////////////// Serial Comm.  //////////////////////////
*/
void setup()
{
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(LEFT_STEER_ANGLE);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  Wire.begin(5);                // join i2c bus with address #9 for Right Motor
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
} 
  
  
void loop()
{

  //Steering_Angle = 90;

 // steering_control(); 
  
 // read_serial_com(); 

  steering_control();
  if(Motor_Speed> 0)      motor_control(1,Motor_Speed);
  else if(Motor_Speed< 0) motor_control(-1,-Motor_Speed);
  else motor_control(0,0);
    
}
