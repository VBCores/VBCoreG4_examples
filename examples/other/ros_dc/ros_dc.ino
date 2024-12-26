#include <VBCoreG4_arduino_system.h>
#include <ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
float linx;
int velocity = 0; 
int forward;; 


void messageCb( const geometry_msgs::Twist& msg){
  linx=  msg.linear.x;
  if(linx > 12) linx = 12;
  else if(linx< -12) linx = -12;
  if (linx>=0) forward = HIGH;
  else forward = LOW;
  velocity = int(abs(linx)*255/12);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13
#define VrefPin PA4

HardwareTimer *timer = new HardwareTimer(TIM3);



void setup() {
 
  pinMode(PB3, OUTPUT); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(VrefPin, OUTPUT);
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VrefPin, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  Serial.begin(115200);

  pinMode(LED2, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  

  timer->pause();
  timer->setOverflow(100, HERTZ_FORMAT); //будем вызывать функцию вращения мотором с частотой 100 Гц
  timer->attachInterrupt(move);
  timer->refresh();
  timer->resume();

  delay(1000);
}

void loop() {  
  str_msg.data = String(velocity).c_str();
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}

void move(){ // на оба вывода при нажатии кнопки подается 0, 
            // поэтому подаем аналоговый сигнал только на один вывод, 
            // в зависимости от направления вращения мотора
  if (forward == HIGH) {
      analogWrite(IN1, 255);
      analogWrite(IN2, 255-velocity); 
    }
  if (forward == LOW) {
    analogWrite(IN2, 255);
    analogWrite(IN1, 255-velocity); 
  }
}