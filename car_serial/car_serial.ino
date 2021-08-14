#include <ros.h>
#include <std_msgs/Int32.h>

#define ENABLE_L 2
#define MOTOR_L_SIG1 3
#define MOTOR_L_SIG2 4
#define ENABLE_R 5
#define MOTOR_R_SIG1 6
#define MOTOR_R_SIG2 7

ros::NodeHandle nh;

void setMotor(int LR, char mode){
  switch(mode){
    case 'f' :
      digitalWrite(LR+1, HIGH);
      digitalWrite(LR+2, LOW);
      break;
    case 'n' :
      digitalWrite(LR+1, LOW);
      digitalWrite(LR+2, LOW);
      break;
    case 'r' :
      digitalWrite(LR+1, LOW);
      digitalWrite(LR+2, HIGH);
      break;
  }
}

void messageCb_motor(const std_msgs::Int32 &motor_msg){

  switch(motor_msg.data){
    case 0 :
      setMotor(ENABLE_L, 'n');
      setMotor(ENABLE_R, 'f');
      break;
    case 1 :
      setMotor(ENABLE_L, 'f');
      setMotor(ENABLE_R, 'f');
      break;
    case 2 :
      setMotor(ENABLE_L, 'f');
      setMotor(ENABLE_R, 'n');
      break;
    case 3 :
      setMotor(ENABLE_L, 'r');
      setMotor(ENABLE_R, 'f');
      break;
    case 4 :
      setMotor(ENABLE_L, 'n');
      setMotor(ENABLE_R, 'n');
      break;
    case 5 :
      setMotor(ENABLE_L, 'f');
      setMotor(ENABLE_R, 'r');
      break;
    case 6 :
      setMotor(ENABLE_L, 'n');
      setMotor(ENABLE_R, 'r');
      break;
    case 7 :
      setMotor(ENABLE_L, 'r');
      setMotor(ENABLE_R, 'r');
      break;
    case 8 :
      setMotor(ENABLE_L, 'r');
      setMotor(ENABLE_R, 'n');
      break;
    default:
      setMotor(ENABLE_L, 'n');
      setMotor(ENABLE_R, 'n');
      break;
  }
}

// サブスクライバー
ros::Subscriber<std_msgs::Int32> sub_motor("motor", &messageCb_motor);

void setup() {

  // DCモーター
  pinMode(ENABLE_L, OUTPUT);
  pinMode(MOTOR_L_SIG1, OUTPUT);
  pinMode(MOTOR_L_SIG2, OUTPUT);
  pinMode(ENABLE_R, OUTPUT);
  pinMode(MOTOR_R_SIG1, OUTPUT);
  pinMode(MOTOR_R_SIG2, OUTPUT);
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);

  // ROS初期設定
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // サブスクライブ実行
  nh.subscribe(sub_motor);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
