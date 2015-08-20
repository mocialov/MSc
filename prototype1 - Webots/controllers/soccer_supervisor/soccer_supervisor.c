/*
 * File:         soccer_supervisor.c
 * Date:         February 11th, 2003
 * Description:  Supevisor the soccer game from soccer.wbt
 *               Send the coordinates and orientations of each robot and the 
 *               coordinates of the ball to each robot via an emitter.       
 * Author:       Olivier Michel
 * Modifications:Simon Blanchoud - September 4th, 2006
 *                - Adapted the code to the Webots standards
 *               Yvan Bourquin - October 2nd, 2007
 *                - Adapted to new packet-oriented emitter/receiver API
 *
 * Copyright (c) 2006 Cyberbotics - www.cyberbotics.com
 */

#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <math.h>
#include <webots/robot_window.h>

#include "fcm.h"


#define ROBOTS 1                /* number of robots */
#define GOAL_X_LIMIT 0.745
#define TIME_STEP 64

#define MIN(x, y) ((x)>(y)?(y):(x))

static int time_step = -1;
WbDeviceTag emitter, receiver;

typedef struct Data
{
   const unsigned char *image_values;
   int image_size;

} ImageData;


static void set_scores(int b, int y) {
  char score[16];

  sprintf(score, "%d", b);
  wb_supervisor_set_label(0, score, 0.92, 0.01, 0.1, 0x0000ff, 0.0); /* blue */
  sprintf(score, "%d", y);
  wb_supervisor_set_label(1, score, 0.05, 0.01, 0.1, 0xffff00, 0.0); /* yellow */
}

float find_max(int m, float data[m]){
  float MAX=0.0;
  int i;
  for(i=0;i<m;i++)
    if(MAX<data[i])
      MAX=data[i];
      
  return MAX;
}

void free_matrix(int **matrix, int size_x)
{
    for(int i = 0; i < size_x; i++)
        free(matrix[i]);
    free(matrix);
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

//check whether slave have sent values to the supervisor
void check_for_slaves_data(w){
    if (wb_receiver_get_queue_length(receiver) > 0) {
      int i, j, m, n;
      const unsigned char *image_values = wb_receiver_get_data(receiver);
      const int received_data_size = wb_receiver_get_data_size(receiver)/sizeof(unsigned char);    
      
     // printf("supervisor\n");
//printf_ByteArray(image_values, 160*120);
      
      ImageData image_data;
      image_data.image_values = image_values;
      image_data.image_size = received_data_size;
      
      wb_robot_window_custom_function(&image_data);

      wb_receiver_next_packet(receiver);
    }
}

int main() {
  printf("hello from supervisor\n");
  
  const char *robot_name[ROBOTS] = {"NAO"};
  WbNodeRef node;
  WbFieldRef robot_translation_field[ROBOTS],robot_rotation_field[ROBOTS],ball_translation_field;
  //WbDeviceTag emitter, receiver;
  int i,j;
  int score[2] = { 0, 0 };
  double time = 10 * 60;    /* a match lasts for 10 minutes */
  double ball_reset_timer = 0;
  double ball_initial_translation[3] = { -2.5, 0.0324568, 0 };
  double robot_initial_translation[ROBOTS][3] = {
      {-4.49515, 0.234045, -0.0112415},
      {0.000574037, 0.332859, -0.00000133636}};
  double robot_initial_rotation[ROBOTS][4] = {
      {0.0604202, 0.996035, -0.0652942, 1.55047},
      {0.000568956, 0.70711, 0.707104, 3.14045}};
  double packet[ROBOTS * 3 + 2];
  char time_string[64];
  const double *robot_translation[ROBOTS], *robot_rotation[ROBOTS], *ball_translation;

  wb_robot_init();
  
  time_step = wb_robot_get_basic_time_step();
  
  emitter = wb_robot_get_device("emitter");
  wb_receiver_enable(emitter, time_step);
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);


  for (i = 0; i < ROBOTS; i++) {
    node = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_translation_field[i] = wb_supervisor_node_get_field(node,"translation");
    robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
    for(j=0;j<3;j++) robot_initial_translation[i][j]=robot_translation[i][j];
    robot_rotation_field[i] = wb_supervisor_node_get_field(node,"rotation");
    robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
    for(j=0;j<4;j++) robot_initial_rotation[i][j]=robot_rotation[i][j];
  }

  node = wb_supervisor_node_get_from_def("BALL");
  ball_translation_field = wb_supervisor_node_get_field(node,"translation");
  ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
  for(j=0;j<3;j++) ball_initial_translation[j]=ball_translation[j];
  /* printf("ball initial translation = %g %g %g\n",ball_translation[0],ball_translation[1],ball_translation[2]); */
  set_scores(0, 0);

  while(wb_robot_step(TIME_STEP)!=-1) {
    //printf("supervisor commands START!\n");
    check_for_slaves_data();
    
    ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
    for (i = 0; i < ROBOTS; i++) {
      robot_translation[i]=wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
      /* printf("coords for robot %d: %g %g %g\n",i,robot_translation[i][0],robot_translation[i][1],robot_translation[i][2]); */
      packet[3 * i]     = robot_translation[i][0];  /* robot i: X */
      packet[3 * i + 1] = robot_translation[i][2];  /* robot i: Z */

      if (robot_rotation[i][1] > 0) {               /* robot i: rotation Ry axis */
        packet[3 * i + 2] = robot_rotation[i][3];   /* robot i: alpha */
      } else { /* Ry axis was inverted */
        packet[3 * i + 2] = -robot_rotation[i][3];   
      }
    }
    packet[3 * ROBOTS]     = ball_translation[0];  /* ball X */
    packet[3 * ROBOTS + 1] = ball_translation[2];  /* ball Z */
    wb_emitter_send(emitter, packet, sizeof(packet));

    /* Adds TIME_STEP ms to the time */
    time -= (double) TIME_STEP / 1000;
    if (time < 0) {
      time = 10 * 60; /* restart */
    }
    sprintf(time_string, "%02d:%02d", (int) (time / 60), (int) time % 60);
    wb_supervisor_set_label(2, time_string, 0.45, 0.01, 0.1, 0x000000, 0.0);   /* black */

    if (ball_reset_timer == 0) {
      if (ball_translation[0] > GOAL_X_LIMIT) {  /* ball in the blue goal */
        set_scores(++score[0], score[1]);
        ball_reset_timer = 3;   /* wait for 3 seconds before reseting the ball */
      } else if (ball_translation[0] < -GOAL_X_LIMIT) {  /* ball in the yellow goal */
        set_scores(score[0], ++score[1]);
        ball_reset_timer = 3;   /* wait for 3 seconds before reseting the ball */
      }
    } else {
      ball_reset_timer -= (double) TIME_STEP / 1000.0;
      if (ball_reset_timer <= 0) {
        ball_reset_timer = 0;
        wb_supervisor_field_set_sf_vec3f(ball_translation_field, ball_initial_translation);
        for (i = 0; i < ROBOTS; i++) {
          wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], robot_initial_translation[i]);
          wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], robot_initial_rotation[i]);
        }
      }
    }
  }
  
  wb_robot_cleanup();

  return 0;
}
