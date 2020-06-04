#include <TimerOne.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#define RATIO 0.05  // increase or decrease ratio of motor pwm
#define MINPWM 129  // minimum pwm of linear motor
#define MAXPWM 220  // maximum pwm of linear motor

double input_pwm = MINPWM;
double input_pwm_real = MINPWM;
double input_pwm_pre = MINPWM;
int flag = 0;
int flag_message = 0;

bool button_down = false;
bool button_up = false;
bool button_shutdown = false;

int DOOR = 0;
int rstatus = 0;

int counter = 0;
bool flag_led = false;

float left[5]{0};  // 0:p, 1:p_pre, 2:v, 3:v_pre, 4:a
float right[5]{0}; // 0:p, 1:p_pre, 2:v, 3:v_pre, 4:a
bool flag_left {0}, flag_right {0};
bool flag_red = false;

ros::NodeHandle nh;

void timerISR();
void button_control();
void message_control();
void action_DOOR(const std_msgs::Int32& msg);
void check_camera(const std_msgs::Int32& msg);
void motor_left(const std_msgs::Float64& msg);
void motor_right(const std_msgs::Float64& msg);

std_msgs::Int32 door_msg;
bool flag_pub = false;
short pub_counter = 0;

ros::Publisher pub_door("doorStatus", &door_msg);
ros::Subscriber<std_msgs::Int32> sub("doorStatus", &action_DOOR);
ros::Subscriber<std_msgs::Int32> sub_camera("cameraperson", &check_camera);
ros::Subscriber<std_msgs::Float64> sub_motor_encoder_left("motor/encoder/left", &motor_left);
ros::Subscriber<std_msgs::Float64> sub_motor_encoder_right("motor/encoder/right", &motor_right);

void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT); // Linear motor control pin
  pinMode(12, INPUT);  // Down button pin
  pinMode(13, INPUT);  // Up button pin
  pinMode(8,  INPUT);  // Shutdown button pin
  pinMode(4, OUTPUT);  // RIGHT GREEN
  pinMode(5, OUTPUT);  // LEFT GREEN
  pinMode(6, OUTPUT);  // RED
  
  nh.initNode();
  nh.advertise(pub_door);
  nh.subscribe(sub);
  nh.subscribe(sub_camera);
  nh.subscribe(sub_motor_encoder_left);
  nh.subscribe(sub_motor_encoder_right);

  Timer1.initialize(2000);
  Timer1.attachInterrupt(timerISR);

  door_msg.data = 0;  // Initialize a variable.

  delay(1000); // Wait for 1sec.
}

void loop() {
  // put your main code here, to run repeatedly:
  button_control();

  message_control();

  // convert variable double to char
  // to display on terminal
  //char input_pwm_c[3];
  //sprintf(input_pwm_c, "%d", int(input_pwm_real));
  //sprintf(input_pwm_c, "%d", rstatus);
  //nh.loginfo(input_pwm_c);

  if(flag_left == true && flag_right == true) flag_red = true;
  else flag_red = false;
  digitalWrite(6, flag_red);
  //digitalWrite(6, HIGH);
  
  if(input_pwm <= MINPWM){
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    if(flag_pub == true){
      pub_door.publish(&door_msg);
      pub_counter++;
      if(pub_counter > 9){
        pub_counter = 0;
        flag_pub = false;
        door_msg.data = 0;
      }         
    }
 
    //digitalWrite(6, HIGH);
  }
  else{
    digitalWrite(4, flag_led);
    digitalWrite(5, flag_led);
    flag_pub = true;
    door_msg.data = 3;
    //digitalWrite(6, HIGH);    
  }

  /*if(flag_pub == true){
    pub_door.publish(&door_msg);
    pub_counter++;
    if(pub_counter > 9){
      counter = 0;
      flag_pub = false;
      door_msg.data = 0;
    }
  }*/
  
  nh.spinOnce();  
  
  delay(10);
}
