#include "nunchuck_funcs.h";

const float deadZoneBuffer = 0.1;

static void setupFunctions(){

  //set up the powerpins and the loop for getting data
  nunchuck_setpowerpins();
  nunchuck_init();
  
  }

static void input(){
  //decode input to data usable by the control program
  nunchuck_get_data();
  int xPushRaw = nunchuck_joyx();
  int yPushRaw = nunchuck_joyy();
  bool zButton = nunchuck_zbutton();
  bool cButton = nunchuck_cbutton();

  float xPush = (xPush/255) -0.5;
  float yPush = (yPush/255) -0.5;
  //No buttons = lateral
  if (abs(xPush) > abs(yPush)){
    //forwards/backwards
    if (xpush>deadZoneBuffer){
      
      }
    
    }
  

  //z-pitch/roll
  
  //c-vertical/yaw
  
  
  }
