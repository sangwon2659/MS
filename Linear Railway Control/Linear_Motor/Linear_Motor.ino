// 가변저항을 이용한 리니어 이송레일 정역제어

/* 라이브러리 include */
#include <HCMotor.h>

/* 모터드라이버 연결핀 */
#define DIR_PIN 8 //스텝모터드라이버 DIR 연결핀
#define CLK_PIN 9 //스텝모터드라이버 CLK 연결핀

/* HCMotor 라이브러리 인스턴스 생성 */
HCMotor HCMotor;

String readString;

void setup() 
{

  Serial.begin(115200);

  /* 라이브러리 초기화 */
  HCMotor.Init();

  /* 모터0을 스텝모터로 설정하고 연결된 핀을 지정 */
  HCMotor.attach(0, STEPPER, CLK_PIN, DIR_PIN);

  /* 모터를 연속동작모드로 설정*/
  HCMotor.Steps(0,CONTINUOUS);
}

void loop() 
{
  while(Serial.available()){
    char c = Serial.read();
    readString += c;
    delay(2);
  }

  if(readString.length()>0){
    int Speed = readString.toInt();
    if(Speed > 0){
      HCMotor.Direction(0, FORWARD);
      HCMotor.DutyCycle(0, Speed);
    }
    else if(Speed < 0){
      Speed = -Speed;
      HCMotor.Direction(0, REVERSE);
      HCMotor.DutyCycle(0, Speed);
    }
    readString = "";
    HCMotor.DutyCycle(0, Speed);
  }
}
