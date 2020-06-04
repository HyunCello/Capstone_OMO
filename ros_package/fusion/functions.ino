void timerISR(){
  if(input_pwm > MAXPWM){
    input_pwm = MAXPWM;
    input_pwm_real = MAXPWM;
    flag_message = 3; // open or close are done.
  }
  else if(input_pwm < MINPWM){
    input_pwm = MINPWM;
    input_pwm_real = MINPWM;
    flag_message = 3;
  }
  analogWrite(11, input_pwm_real);
  
  counter++;
  if(counter >= 3500) // 7 seconds
    counter = 0;
  if(counter % 250 == 0)
    flag_led ^= 1;

  /*if(input_pwm_pre > MINPWM){
    if(input_pwm <= MINPWM + 1){
      door_msg.data = 3;
      flag_pub = true;      
    }
  }*/
  //else door_msg.data = 0;
}

void button_control(){
  button_up = digitalRead(13);
  button_down = digitalRead(12);
  button_shutdown = digitalRead(8);

  if(button_up == true || button_down == true || button_shutdown == true){
    if(flag_message != 0) flag_message = 0;
    if(button_up){
      flag = 1;
      input_pwm += RATIO;
    }
    else if(button_down){
      flag = 2;
      input_pwm -= RATIO;
    }
    else if(button_shutdown) flag_message = 2;//input_pwm = 127;
  }
  else flag = 0;

  if(flag == 1) input_pwm_real = 250;
  else if(flag == 2) input_pwm_real = MINPWM;
  else input_pwm_real = input_pwm;
}

void message_control(){
  if(flag_message == 1){ // 1 : open
    input_pwm_real = 250;
    input_pwm += RATIO;
  }
  else if(flag_message == 2){ // 2 : close
    input_pwm_real = MINPWM;
    input_pwm -= RATIO;
  }
}
