// 가변저항을 이용한 리니어 이송레일 정역제어

/* 라이브러리 include */
#include "HCMotor.h"
#include <ros.h>
#include <std_msgs/Int32.h>

/* 모터드라이버 연결핀 */
#define DIR_PIN 8 //스텝모터드라이버 DIR 연결핀
#define CLK_PIN 9 //스텝모터드라이버 CLK 연결핀

/* HCMotor 라이브러리 인스턴스 생성 */
HCMotor HCMotor;

ros::NodeHandle nh;

int var;

void messageCb(const std_msgs::Int32 &msg)
{
    var = msg.data;
    if(var > 0){
      HCMotor.Direction(0, FORWARD);
      HCMotor.DutyCycle(0, var);
    }
    else if(var < 0){
      var = -(var);
      HCMotor.Direction(0, REVERSE);
      HCMotor.DutyCycle(0, var);
    }
    HCMotor.DutyCycle(0, var);
    //Serial.println(var);
}

ros::Subscriber<std_msgs::Int32> sub("HCmotor", &messageCb);

void setup() 
{
  //Serial.begin(57600);

  nh.initNode();
  
  /* 라이브러리 초기화 */
  HCMotor.Init();

  /* 모터0을 스텝모터로 설정하고 연결된 핀을 지정 */
  HCMotor.attach(0, STEPPER, CLK_PIN, DIR_PIN);

  /* 모터를 연속동작모드로 설정*/
  HCMotor.Steps(0,CONTINUOUS);

  nh.subscribe(sub);
  HCMotor.DutyCycle(0,0);
}

void loop() 
{
  nh.spinOnce();
  delay(10);
}
