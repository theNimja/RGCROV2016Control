//Robert gordons College ROV Contest
//Control Module code
//By Nimrod Libman, 16/06/2016



const float deadZoneBuffer = 0.1; //if the absolute values of the x and y postion of the stick are withing this value of each other, stick is in deadzone.

int vertIntensity; //PWM intensity for vertical motors. Set by input() every tick.
int lateIntensity; //PWM intensity for lateral  motors. Set by input() every tick.
char vertMotorDir; //determines whether we are going up, down, pitching up, pitching down, or rolling left or right. set by input() every tick.
char lateMotorDir; //determines whether we are going forward, back, left, right, or yawing left or right. set by input() every tick.


//  DIAGRAM OF JOYSTICK
//       +1
//      _______
//     / \   /  \
//    /   \ /    \
// -1|     .     |+1
//   |    / \    |
//    \  /   \  /
//     \_______/
//        -1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  input();//defined by the currently set input library.
}
