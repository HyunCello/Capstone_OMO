void action_DOOR(const std_msgs::Int32& msg){
  DOOR =msg.data;

  if(DOOR == 2/*"close"*/){
    flag_message = 2;
    //input_pwm = 130;
  }
  if(DOOR == 1/*"open"*/){
    flag_message = 1;
    //input_pwm = 250;
  }
}

void check_camera(const std_msgs::Int32& msg){
  rstatus = msg.data;

  if(rstatus == 1) flag_message = 1;
  else if(rstatus == 2) flag_message = 2;
}

void motor_left(const std_msgs::Float64& msg){
  left[0] = msg.data;           // position
  left[2] = left[0] - left[1];  // velocity
  left[4] = left[2] - left[3];  // acceleration
  left[1] = left[0];            // save current position
  left[3] = left[2];            // save current velocity

  //if(left[2] * left[4] < 0) flag_left = true;
  if(left[2] == 0) flag_left = true;
  else flag_left = false;
}
void motor_right(const std_msgs::Float64& msg){
  right[0] = msg.data;           // position
  right[2] = right[0] - right[1];  // velocity
  right[4] = right[2] - right[3];  // acceleration
  right[1] = right[0];            // save current position
  right[3] = right[2];            // save current velocity

  //if(right[2] * right[4] < 0) flag_right = true;
  if(right[2] == 0) flag_right = true;
  else flag_right = false;
}
