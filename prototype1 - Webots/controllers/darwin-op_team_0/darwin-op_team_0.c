#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/led.h>
#include <math.h>

#define TIME_STEP 40
#define NSERVOS   20

static const char *servo_names[NSERVOS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

static double amplitudes[6] = {
  0.2, -0.2, 0.5, -0.5, 0.5, -0.5
};

static double offsets[6] = {
  2.5, -2.5, -0.5, 0.5, -1.0, 1.0
};

static double frequency = 1.0;

int main(int argc, char **argv)
{
  wb_robot_init();
  
  int i;
  WbDeviceTag servos[NSERVOS];
  WbDeviceTag head_led;
  
  for (i=0; i<NSERVOS; i++)
    servos[i] = wb_robot_get_device(servo_names[i]);
  head_led = wb_robot_get_device("HeadLed");
  wb_led_set(head_led, 0x40C040);
  
  do {
    double t = wb_robot_get_time();
    for (i=0; i<6; i++)
      wb_servo_set_position(servos[i], amplitudes[i]*sin(frequency*t) + offsets[i]);
    
  } while (wb_robot_step(TIME_STEP) != -1);
  
  wb_robot_cleanup();
  
  return 0;
}

