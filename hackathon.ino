#include<Wire.h> //가속도 센서를 제어하기 위한 라이브러리
#include <SoftwareSerial.h> 

SoftwareSerial BTSerial(2,3); //ble 핀 번호 지정

const int MPU=0x68;  //MPU 6050 의 I2C 기본 주소
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //가속도 센서 변수 지정

int IRpin = 0;
int lat=37.514980,lon=127.063237; //위도와 경도(jbk 컨벤션 기준)
int r = 6378137; //지구 반지름
int angle=90; // 진행방향 임의 지정

void setup() {
  Wire.begin();      //Wire 라이브러리 초기화
  Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //MPU-6050 시작 모드로
  Wire.endTransmission(true); 
  
  Serial.begin(9600);
  BTSerial.begin(9600); 
   
  delay(1000);

  BTSerial.write("AT"); //AT커맨드 확인
  delay(1000);
  while (BTSerial.available()){
     Serial.write(BTSerial.read()); //확인이 되면 데이터 출력
  }
  
  BTSerial.write("AT+CON74DAEAB1B7C3"); //해당 블루투스 맥주소와 연결
  delay(2000);
  while (BTSerial.available()){
     Serial.write(BTSerial.read()); //연결이 되면 데이터 출력
  }
  
}

void loop() {
  
  int x,y; //GPS의 위도와 경도를 x,y축으로 변경하는 변수
  int temp =0; //차간 사이의 rssi값을 저장하기위한 임시변수
  double dis; //차간 사이의 거리를 저장하는 변수
  
  BTSerial.write("AT+RSSI?"); //연결된 블루투스 간의 rssi 값 출력
  delay(300);
  while (BTSerial.available()){
    if(BTSerial.find("-")){
      temp = BTSerial.parseInt(); //문자열로 출력되는 데이터중 rssi 값만을 추출하여 그값을 정수형으로 저장 
    }
  }
  BTSerial.print("RSSI : ");
  BTSerial.println((-temp)); //rssi 값 출력
  dis = calculateDistance(-65, temp); //차간 사이의 거리를 계산하는 calculateDistance 출력
  BTSerial.print("Distance : ");
  BTSerial.println(dis); //차간 사이 거리 출력
  
  float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float distance = 65*pow(volts, -1.10); //초음파 센서의 거리 저장
  BTSerial.print("MW : ");
  BTSerial.println(distance); //초음파 센서 거리 출력
  
  x = r * lon * cos(180/PI*lat); //gps 의 위도와 경도를 x,y축으로 변경하기 위한 공식
  y = r * lat;
  BTSerial.print("x axis : ");
  BTSerial.print(x+dis*sin(angle)); //터널안의 차의 x,y축을 나타내기 위한 계산
  BTSerial.print(" y axis : ");
  BTSerial.println(y+dis*cos(angle));
  
  Wire.beginTransmission(MPU);    //데이터 전송시작
  Wire.write(0x3B);               // register 0x3B (ACCEL_XOUT_H), 큐에 데이터 기록
  Wire.endTransmission(false);    //연결유지
  Wire.requestFrom(MPU,14,true);  //MPU에 데이터 요청
  //데이터 한 바이트 씩 읽어서 반환
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  BTSerial.print("AcY = "); 
  BTSerial.println(AcY); //y축 가속도 출력
  

  if(distance<41){ //초음파 센서가 일정거리 이하일 때
    BTSerial.println("Warning Fall of Rocks" ); //초음파 낙석
  }
  if(AcY>=15500&&AcY<=16250){ //가속도 값이 일정값 이상일 때
    BTSerial.println("Warning Braking!!!" ); //멈췄을때
  }
  
  BTSerial.println();
  delay(2000);
}

double calculateDistance(int txPower, double rssi) { //rssi 값을 이용하여 차간 사이의 거리를 구하는 함수
  if (rssi == 0) { //블루투스가 통신이 되지 않을때
    return -1.0; 
  }
  double ratio = rssi*1.0/txPower;
  if (ratio < 1.0) {
    return pow(ratio,10);
  }
  else {
    double accuracy =  (0.89976)*pow(ratio,7.7095) + 0.111;    
    return accuracy;
  }
}
