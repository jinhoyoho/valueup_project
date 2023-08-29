#include <ros.h>
#include <geometry_msgs/Twist.h>


// ROSCORE 켜고
// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
// 높이는 rostopic pub /cmd_vel geometry_msgs/Twist 리니어x +-0.5

ros::NodeHandle nh;

int PUL = 7; // Define Pulse pin
int DIR = 6; // Define Direction pin
int ENA = 5; // Define Enable Pin

void motorCallback(const geometry_msgs::Twist& msg);

ros::Subscriber<const geometry_msgs::Twist> sub("/cmd_vel", motorCallback);

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(ENA, LOW);  // Enable 초기 설정 (비활성화)

  nh.initNode();  // ROS 노드 초기화

  nh.subscribe(sub);  // /motor_command 토픽 subscribe
  nh.negotiateTopics();
}

void loop() {
  nh.spinOnce();  // ROS 노드 처리

  delay(10);
}

void moveSteps(int steps, int dir) {
  digitalWrite(DIR, dir);  // 방향 설정

  for (int i = 0; i < steps; i++) {
    digitalWrite(ENA, HIGH); // 활성화
    digitalWrite(PUL, HIGH);
    delayMicroseconds(100);
    digitalWrite(PUL, LOW);
    delayMicroseconds(100);
  }
}


void motorCallback(const geometry_msgs::Twist& msg) {
  if (msg.linear.x == 0.5) {
    moveSteps(5000, HIGH);  // 상승
  } else if (msg.linear.x == -0.5) {
    moveSteps(5000, LOW); // 하강
  } else {
    digitalWrite(DIR, LOW);
    digitalWrite(ENA, HIGH);
    digitalWrite(PUL, LOW);
  }
}
