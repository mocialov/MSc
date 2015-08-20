#include <stdio.h>
#include <webots/robot.h>
#include <webots/utils/motion.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <webots/servo.h> 
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/led.h>
#include <string.h>
#include <stdlib.h>

WbDeviceTag receiver, emitter;

#define PHALANX_MAX 8

static int time_step = -1;

// simulated devices
static WbDeviceTag CameraTop, CameraBottom, LaserHead;            // cameras
static WbDeviceTag us[4];                              // ultra sound sensors
static WbDeviceTag accelerometer, gyro, inertial_unit; // inertial unit
static WbDeviceTag fsr3d[2];                           // force sensitive resistors
static WbDeviceTag lfoot_lbumper, lfoot_rbumper;       // left foot bumpers
static WbDeviceTag rfoot_lbumper, rfoot_rbumper;       // right foot bumpers
static WbDeviceTag leds[7];                            // controllable led groupsstatic WbDeviceTag lphalanx[PHALANX_MAX];
static WbDeviceTag rphalanx[PHALANX_MAX];              // right hand motors
static WbDeviceTag lphalanx[PHALANX_MAX];              // left hand motors
static WbDeviceTag RShoulderPitch;
static WbDeviceTag LShoulderPitch;
static WbDeviceTag HeadYaw;
static WbDeviceTag HeadPitch;


static void find_and_enable_devices() {

  // camera
  CameraTop = wb_robot_get_device("CameraTop");
  CameraBottom = wb_robot_get_device("CameraBottom");
  wb_camera_enable(CameraTop, 4 * time_step);
  wb_camera_enable(CameraBottom, 4 * time_step);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // gyro
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);
  
  // inertial unit
  inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  // ultrasound sensors
  us[0] = wb_robot_get_device("USSensor1");
  us[1] = wb_robot_get_device("USSensor2");
  us[2] = wb_robot_get_device("USSensor3");
  us[3] = wb_robot_get_device("USSensor4");
  int i;
  for (i = 0; i < 4; i++)
    wb_distance_sensor_enable(us[i], time_step);

  // foot sensors
  fsr3d[0] = wb_robot_get_device("LFsr");
  wb_touch_sensor_enable(fsr3d[0], time_step);
  fsr3d[1] = wb_robot_get_device("RFsr");
  wb_touch_sensor_enable(fsr3d[1], time_step);

  // foot bumpers
  lfoot_lbumper = wb_robot_get_device("BumperLFootLeft");
  lfoot_rbumper = wb_robot_get_device("BumperLFootRight");
  rfoot_lbumper = wb_robot_get_device("BumperRFootLeft");
  rfoot_rbumper = wb_robot_get_device("BumperRFootRight");
  wb_touch_sensor_enable(lfoot_lbumper, time_step);
  wb_touch_sensor_enable(lfoot_rbumper, time_step);
  wb_touch_sensor_enable(rfoot_lbumper, time_step);
  wb_touch_sensor_enable(rfoot_rbumper, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");
  
  // get phalanx motor tags
  // the real Nao has only 2 motors for RHand/LHand
  // but in Webots we must implement RHand/LHand with 2x8 motors
  for (i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, "LPhalanx%d", i + 1);
    lphalanx[i] = wb_robot_get_device(name);
    sprintf(name, "RPhalanx%d", i + 1);
    rphalanx[i] = wb_robot_get_device(name);
  }
  
  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");
  
  // shoulder pitch motors
  HeadYaw = wb_robot_get_device("HeadYaw");
  HeadPitch = wb_robot_get_device("HeadPitch");
  
  //LaserHead = wb_robot_get_device("URG-04LX-UG01");
  //wb_camera_enable(LaserHead, 4 * time_step);

  // keyboard
  //wb_robot_keyboard_enable(10 * time_step); 
}

int printf_ByteArray(const unsigned char *data, size_t len) {
  size_t i;
  int result = 0;
  for (i = 0; i < len; i++) {
    int y;
    int ch = data[i];
    static const char escapec[] = "\a\b\t\n\v\f\n\'\"\?\\";
    char *p = strchr(escapec, ch);
    if (p && ch) {
      static const char escapev[] = "abtnvfn\'\"\?\\";
      y = printf("\\%c", escapev[p - escapec]);
    } else if (isprint(ch)) {
      y = printf("%c", ch);
    } else {
      // If at end of array, assume _next_ potential character is a '0'.
      int nch = i >= (len - 1) ? '0' : data[i + 1];
      if (ch < 8 && (nch < '0' || nch > '7')) {
        y = printf("\\%o", ch);
      } else if (!isxdigit(nch)) {
        y = printf("\\x%X", ch);
      } else {
        y = printf("\\o%03o", ch);
      }
    }
    if (y == EOF)
      return EOF;
    result += y;
  }
  return result;
}

void report_step_state_to_supervisor(){
    // send message to supervisor
    
      int width = wb_camera_get_width(CameraTop);
  int height = wb_camera_get_height(CameraTop);

  // read rgb pixel values from the camera
  const unsigned char *image = wb_camera_get_image(CameraTop);
  
 // printf("player\n");
//printf_ByteArray(image, width*height);
  
    wb_emitter_send(emitter, image, height * width * 8);
    
    //free(data_values);
}

int main() {
  wb_robot_init();
  
  printf("hello from NAO\n");
  
  time_step = wb_robot_get_basic_time_step();
  

  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  emitter = wb_robot_get_device("emitter2");
  wb_receiver_enable(emitter, time_step);
  
  
  find_and_enable_devices();
  
  wb_motor_set_position (RShoulderPitch, 1.57079633);
  wb_motor_set_position (LShoulderPitch, 1.57079633);
  wb_motor_set_position (HeadYaw, 0.0);
  //wb_motor_set_position (HeadPitch, 0.1); //10 degrees




  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
  

      //check_for_new_genes();
      //sense_compute_and_actuate();
      //int samples = wb_camera_get_width(LaserHead);
      //double field_of_view = wb_camera_get_fov(LaserHead);
      //const float *values = wb_camera_get_range_image(LaserHead);
      
      report_step_state_to_supervisor();
      
      //printf("%f\n", field_of_view);
      //printf("size: %d\n", sizeof(values));
      //int i;
      //for(i=0; i<samples;i++){
      //  printf("value %d is %f\n",i, values[i]);
      //}
      
      if(wb_robot_step(time_step) == 0){
          //report_step_state_to_supervisor();
      }
  }

  wb_robot_cleanup();

  return 0;
}
