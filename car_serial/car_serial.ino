#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>

#define REAR_LED_L 8
#define REAR_LED_R 9
#define FRONT_LED_L 10
#define FRONT_LED_R 11
#define SERVO_PIN 6
#define MOTOR_SIG1 3
#define MOTOR_SIG2 4
#define ENABLE 5

Servo myServo;
ros::NodeHandle nh;
bool ledControlleFlag = true;

void messageCb_led(const std_msgs::Int32 &led_msg){
  if(led_msg.data){
    ledControlleFlag=false;
    digitalWrite(REAR_LED_L, HIGH);
    digitalWrite(REAR_LED_R, HIGH);
    digitalWrite(FRONT_LED_L, HIGH);
    digitalWrite(FRONT_LED_R, HIGH);
  }else{
    ledControlleFlag=true;
    digitalWrite(REAR_LED_L, LOW);
    digitalWrite(REAR_LED_R, LOW);
    digitalWrite(FRONT_LED_L, LOW);
    digitalWrite(FRONT_LED_R, LOW);
  }
}

void messageCb_servo(const std_msgs::Int32 &servo_msg){
  if(65<=servo_msg.data && servo_msg.data<=115){
    myServo.write(servo_msg.data);

    if(ledControlleFlag){
      if(servo_msg.data<90) {
        digitalWrite(REAR_LED_L, HIGH);
        digitalWrite(REAR_LED_R, LOW); 
      }else if(90<servo_msg.data) {
        digitalWrite(REAR_LED_R, HIGH);
        digitalWrite(REAR_LED_L, LOW); 
      }else {
        digitalWrite(REAR_LED_L, LOW); 
        digitalWrite(REAR_LED_R, LOW);
      }
    }
  }
}

void messageCb_motor(const std_msgs::Int32 &motor_msg){
  if(motor_msg.data == 1){
    // 前進
    digitalWrite(MOTOR_SIG1, HIGH);
    digitalWrite(MOTOR_SIG2, LOW);
    digitalWrite(ENABLE, HIGH);
    // 前LED点灯
    if(ledControlleFlag){
      digitalWrite(FRONT_LED_L, HIGH);
      digitalWrite(FRONT_LED_R, HIGH);
    }
  }else if(motor_msg.data == -1){
    // 後退
    digitalWrite(MOTOR_SIG1, LOW);
    digitalWrite(MOTOR_SIG2, HIGH);
    digitalWrite(ENABLE, HIGH);
    // 前LED消灯
    if(ledControlleFlag){
      digitalWrite(FRONT_LED_L, LOW);
      digitalWrite(FRONT_LED_R, LOW);
    }
    
  }else{
    // ストップ
    digitalWrite(MOTOR_SIG1, LOW);
    digitalWrite(MOTOR_SIG2, LOW);
    digitalWrite(ENABLE, LOW);
    // 前LED消灯
    if(ledControlleFlag){
      digitalWrite(FRONT_LED_L, LOW);
      digitalWrite(FRONT_LED_R, LOW);
    }
  }
}

// サブスクライバー
ros::Subscriber<std_msgs::Int32> sub_led("led", &messageCb_led);
ros::Subscriber<std_msgs::Int32> sub_servo("servo", &messageCb_servo);
ros::Subscriber<std_msgs::Int32> sub_motor("motor", &messageCb_motor);

void setup() {
  // LED
  pinMode(REAR_LED_L,  OUTPUT);
  pinMode(REAR_LED_R,  OUTPUT);
  pinMode(FRONT_LED_L,  OUTPUT);
  pinMode(FRONT_LED_R,  OUTPUT);

  // サーボ
  myServo.attach(SERVO_PIN);
  myServo.write(90);

  // DCモーター
  pinMode(MOTOR_SIG1, OUTPUT);
  pinMode(MOTOR_SIG2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // ROS初期設定
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // サブスクライブ実行
  nh.subscribe(sub_led);
  nh.subscribe(sub_servo);
  nh.subscribe(sub_motor);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
