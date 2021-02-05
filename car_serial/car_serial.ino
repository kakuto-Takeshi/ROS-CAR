#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>

#define SERVO_PIN 6
#define MOTOR_SIG1 3
#define MOTOR_SIG2 4
#define ENABLE 5

Servo myServo;
const int OFFSET = 7;
ros::NodeHandle nh;

void messageCb_servo(const std_msgs::Int32 &servo_msg){
  if(65<=servo_msg.data && servo_msg.data<=115){
    myServo.write(servo_msg.data+OFFSET);
  }
}

void messageCb_motor(const std_msgs::Int32 &motor_msg){
  if(motor_msg.data == 1){
    // 前進
    digitalWrite(MOTOR_SIG1, HIGH);
    digitalWrite(MOTOR_SIG2, LOW);
    digitalWrite(ENABLE, HIGH);
  }else if(motor_msg.data == -1){
    // 後退
    digitalWrite(MOTOR_SIG1, LOW);
    digitalWrite(MOTOR_SIG2, HIGH);
    digitalWrite(ENABLE, HIGH);
  }else{
    // ストップ
    digitalWrite(MOTOR_SIG1, LOW);
    digitalWrite(MOTOR_SIG2, LOW);
    digitalWrite(ENABLE, LOW);
  }
}

// サブスクライバー
ros::Subscriber<std_msgs::Int32> sub_servo("servo", &messageCb_servo);
ros::Subscriber<std_msgs::Int32> sub_motor("motor", &messageCb_motor);

void setup() {

  // サーボ
  myServo.attach(SERVO_PIN);
  myServo.write(90+OFFSET);

  // DCモーター
  pinMode(MOTOR_SIG1, OUTPUT);
  pinMode(MOTOR_SIG2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // ROS初期設定
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // サブスクライブ実行
  nh.subscribe(sub_servo);
  nh.subscribe(sub_motor);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
