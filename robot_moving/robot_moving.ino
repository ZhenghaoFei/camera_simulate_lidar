
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
// based on the sheild of motor I used
int LEFT_MOTOR_DIR_PIN = 7;
int LEFT_MOTOR_PWM_PIN = 6;
int RIGHT_MOTOR_DIR_PIN = 4;
int RIGHT_MOTOR_PWM_PIN = 5;

int linear_movePWM = 200;
int angular_rotPWM = 200;
void messageCb(const geometry_msgs::Twist& msg){
    if(msg.linear.x == 2)
      move_forward();
    if(msg.linear.x == -2)
      move_backward();
    if((msg.linear.x == 0)&&(msg.angular.z == 2))
      turn_left();
    if((msg.linear.x == 0)&&(msg.angular.z == -2))
      turn_right();
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel",&messageCb);
void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  digitalWrite(13, HIGH);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
void setMotorPins(){
   // enable controller pins
   pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT );  //set the motor pins to outputs
   pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT );
   pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT );
   pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT );
}
void move_forward(){
   // specify the direction
   digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
   digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH); 
   analogWrite(LEFT_MOTOR_PWM_PIN, linear_movePWM);
   analogWrite(RIGHT_MOTOR_PWM_PIN, linear_movePWM);
   delay(800);
   stopRobot();
}
void move_backward(){
   // specify the direction
   digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
   digitalWrite(LEFT_MOTOR_DIR_PIN, LOW); 
   analogWrite(LEFT_MOTOR_PWM_PIN, linear_movePWM);
   analogWrite(RIGHT_MOTOR_PWM_PIN, linear_movePWM);
   delay(800);
   stopRobot();
}
void turn_left(){
   // specify the direction
   digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
   digitalWrite(LEFT_MOTOR_DIR_PIN, LOW); 
   analogWrite(LEFT_MOTOR_PWM_PIN, angular_rotPWM);
   analogWrite(RIGHT_MOTOR_PWM_PIN, angular_rotPWM);
   delay(1000);
   stopRobot();
}
void turn_right(){
   // specify the direction
   digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
   digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH); 
   analogWrite(LEFT_MOTOR_PWM_PIN, angular_rotPWM);
   analogWrite(RIGHT_MOTOR_PWM_PIN, angular_rotPWM);
   delay(1000);
   stopRobot();
}
void stopRobot()
{
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}

