//--------------------------------------------------------------------------------------------
//  File:         nao_soccer_supervisor.c
//  Project:      Robotstadium, the online robot soccer competition
//  Description:  Supervisor controller for Robostadium/Nao soccer worlds
//                (You do not need to modify this file for the Robotstadium competition)
//                This controller has several purposes: control the current game state,
//                count the goals, display the scores, move robots and ball to kick-off
//                position, simulate the RoboCup game controller by sending data
//                to the players every 500 ms, check "kick-off shots", throw in ball
//                after it left the field, record match video, penalty kick shootout ...
//  Author:       Olivier Michel & Yvan Bourquin, Cyberbotics Ltd.
//  Date:         July 13th, 2008
//  Changes:      November 6, 2008:  Adapted to Webots 6 API by Yvan Bourquin
//                February 26, 2009: Added throw-in collision avoidance (thanks to Giuseppe Certo)
//                April 23, 2009:    Added: robot is held in position during the INITIAL and SET states
//                May 27, 2009:      Changed penalty kick rules according to latest SPL specification
//                                   Changed field dimensions according to latest SPL specification
//                                   Modified collision detection to support asymmetrical ball
//                May 3, 2010        Modified penalty box from 3000 to 2200 mm
//                                   Modified to avoid swapping controllers
//                April 7, 2011      Adapted to 4 players per team (according to SPL 2011)
//--------------------------------------------------------------------------------------------

#include "RoboCupGameControlData.h"
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

// used for indexing arrays
enum { X, Y, Z };

// max num robots
#define MAX_NUM_ROBOTS (2*MAX_NUM_PLAYERS)

// field dimensions (in meters) according to the SPL Nao Rule Book
#define FIELD_SIZE_X         6.050   // official size of the field
#define FIELD_SIZE_Z         4.050   // for the 2008 competition
#define CIRCLE_DIAMETER      1.250   // soccer field's central circle
#define PENALTY_SPOT_DIST    1.825   // distance between penalty spot and goal
#define PENALTY_AREA_Z_DIM   2.250   // changed according to 2010 specification
#define PENALTY_AREA_X_DIM   0.650
#define GOAL_WIDTH           1.400
#define THROW_IN_LINE_LENGTH 4.000   // total length
#define THROW_IN_LINE_OFFSET 0.400   // offset from side line
#define LINE_WIDTH           0.050   // white lines
#define BALL_RADIUS          0.0325
#define PI 3.141592653589793

//counter for emotion
static int emoCount = 0, recQuant = 1, curEventID = 0;
static double pleasNum = 0, goalPleas = 0, arouNum = 0, goalArou, kickDist = 0.15;
static double kickAngle;
static bool goalFlag = false, 
            defFlag = false, 
            outFlag = false,
            missFlag = false, 
            rPost = false,
            lPost = false;
static double curEvent[5] = {-1, -1, -1, -1, -1};

// throw-in lines
const double THROW_IN_LINE_X_END = THROW_IN_LINE_LENGTH / 2;
const double THROW_IN_LINE_Z = FIELD_SIZE_Z / 2 - THROW_IN_LINE_OFFSET;

// ball position limits
static const double CIRCLE_RADIUS_LIMIT = CIRCLE_DIAMETER / 2 + BALL_RADIUS;
static const double FIELD_X_LIMIT = FIELD_SIZE_X / 2 + BALL_RADIUS;
static const double FIELD_Z_LIMIT = FIELD_SIZE_Z / 2 + BALL_RADIUS;

// penalties
static const double PENALTY_BALL_X_POS = FIELD_SIZE_X / 2 - PENALTY_SPOT_DIST;
static const double PENALTY_GOALIE_X_POS = (FIELD_SIZE_X - LINE_WIDTH) / 2;

// timing
static const int timeCount_STEP = 40;              // should be a multiple of WorldInfo.basictimeCountSTep
static const double MAX_timeCount = 10.0 * 60.0;   // a match half lasts 10 minutes

// indices of the two robots used for penalty kick shoot-out
static const int GOALIE = 0;
static const int ATTACKER = 1;

// robot model
static const int NAO = 0;
static const int DARWIN = 1;

// waistband colors
const double PINK[3] = { 0.9, 0.5, 0.5 };
const double BLUE[3] = { 0.0, 0.0, 1.0 };

// the information we need to keep about each robot
typedef struct {
  WbFieldRef translation;        // to track robot's position
  WbFieldRef rotation;           // to track robot's rotation
  WbFieldRef waist_band_color;   // to be changed during half-timeCount
  int model;                     // NAO or DARWIN
  const double *position;        // pointer to current robot position
} Robot;

// the Robots
static Robot *robots[MAX_NUM_ROBOTS];

// zero vector
static const double ZERO_VEC_3[3] = { 0, 0, 0 };

// global variables
static WbFieldRef ball_translation = NULL;        // to track ball position
static WbFieldRef ball_rotation = NULL;           // to reset ball rotation
static WbDeviceTag emitter;                       // to send game control data to robots
static WbDeviceTag receiver;                      // to receive 'move' requests
static const double *ball_pos = ZERO_VEC_3;       // current ball position (pointer to)
static double timeCount;                               // timeCount [seconds] since end of half game
static int step_count = 0;                        // number of steps since the simulation started
static int last_touch_robot_index = -1;           // index of last robot that touched the ball
static const char *message;                       // instant message: "Goal!", "Out!", etc.
static double message_steps = 0;                  // steps to live for instant message

// Robotstadium match type
enum {
  DEMO,       // DEMO, does not make a video, does not terminate the simulator, does not write scores.txt
  ROUND,      // Regular Robostadium round: 2 x 10 min + possible penalty shots
  FINAL       // Robotstadium final: add sudden death-shots if games is tied after penalty kick shoot-out
};
static int match_type = DEMO;

// default team names displayed by the Supervisor
static char team_names[2][64] = { "Team-0", "Team-1" };

// RoboCup GameController simulation
static struct RoboCupGameControlData control_data; 

// to enforce the "kick-off shot cannot score a goal" rule:
enum {
  KICK_OFF_INITIAL,      // the ball was just put in the central circle center
  KICK_OFF_LEFT_CIRCLE,  // the ball has left the central circle
  KICK_OFF_OK            // the ball was hit by a kick-off team robot after having left the central circle
};
static int kick_off_state = KICK_OFF_INITIAL;

double getAbs(double num){
  if (num < 0)
    return num * -1;
  else
    return num;
}

static int robot_get_teamID(int robot_index) {
  return robot_index < MAX_NUM_PLAYERS ? 0 : 1;
}

static int robot_is_blue(int robot_index) {
  return control_data.teams[TEAM_BLUE].teamNumber == robot_get_teamID(robot_index);
}

static int robot_is_red(int robot_index) {
  return control_data.teams[TEAM_RED].teamNumber == robot_get_teamID(robot_index);
}

static int robot_get_index(int robotID, int teamID) {
  if (robotID >= 0 && robotID < MAX_NUM_PLAYERS && teamID >= 0 && teamID < 2)
    return MAX_NUM_PLAYERS * teamID + robotID;
  else
    return -1;
}

// returns 1 if team 0 plays TEAM_BLUE and team 1 plays TEAM_RED
// returns 0 if team 0 plays TEAM_RED and team 1 playes TEAM_BLUE
static int teams_swapped() {
  return control_data.teams[TEAM_BLUE].teamNumber == 1;
}

static int get_blue_robot_index(int playerID) {
  return playerID + (teams_swapped() ? MAX_NUM_PLAYERS : 0);
}

static int get_red_robot_index(int playerID) {
  return playerID + (teams_swapped() ? 0 : MAX_NUM_PLAYERS);
}

static const char *get_team_name(int team_color) {
  return team_names[control_data.teams[team_color].teamNumber];
}

// create and initialize a Robot struct
static Robot *robot_new(WbNodeRef node) {
  Robot *robot = malloc(sizeof(Robot));
  robot->translation = wb_supervisor_node_get_field(node, "translation");
  robot->rotation = wb_supervisor_node_get_field(node, "rotation");
  robot->waist_band_color = wb_supervisor_node_get_field(node, "waistBandColor");
  robot->position = NULL;
  
  // find robot model
  WbFieldRef controllerField = wb_supervisor_node_get_field(node, "controller");
  const char *controller = wb_supervisor_field_get_sf_string(controllerField);
  if (strstr(controller, "nao"))
    robot->model = NAO;
  else
    robot->model = DARWIN;

  return robot;
}

static const char *get_robot_def_name(int robot_index) {
  static char defname[64];
  int playerID = robot_index % MAX_NUM_PLAYERS;
  int teamID = robot_get_teamID(robot_index);
  
  if (playerID == GOALIE)
    sprintf(defname, "GOAL_KEEPER_%d", teamID);
  else
    sprintf(defname, "PLAYER_%d_%d", playerID, teamID);

  return defname;
}

static void display() {

  const double FONT_SIZE = 0.15;

  // display team names and current score
  char text[64];
  sprintf(text, "%s - %d", get_team_name(TEAM_BLUE), control_data.teams[TEAM_BLUE].score);
  wb_supervisor_set_label(0, text, 0.05, 0.03, FONT_SIZE, 0x0000ff, 0.0); // red
  sprintf(text, "%d - %s", control_data.teams[TEAM_RED].score, get_team_name(TEAM_RED));
  wb_supervisor_set_label(1, text, 0.99 - 0.025 * strlen(text), 0.03, FONT_SIZE, 0xec0f0f, 0.0); // blue

  // display game state or remaining timeCount
  if (control_data.state == STATE_PLAYING)
    sprintf(text,"%02d:%02d",(int)(timeCount/60),(int)timeCount%60);
  else {
    static const char *STATE_NAMES[5] = { "INITIAL", "READY", "SET", "PLAYING", "FINISHED" };
    sprintf(text, "%s", STATE_NAMES[control_data.state]);
  }
  wb_supervisor_set_label(2, text, 0.51 - 0.015 * strlen(text), 0.1, FONT_SIZE, 0x000000, 0.0); // black

  // display instant message
  if (message_steps > 0)
    wb_supervisor_set_label(3, message, 0.51 - 0.015 * strlen(message), 0.9, FONT_SIZE, 0x000000, 0.0); // black
  else {
    // remove instant message
    wb_supervisor_set_label(3, "", 1, 0.9, FONT_SIZE, 0x000000, 0.0);
    message_steps = 0;
  }
}

// add an instant message
static void show_message(const char *msg) {
  message = msg;
  message_steps = 4000 / timeCount_STEP;  // show message for 4 seconds
  display();
  printf("%s\n", msg);
}

static void sendGameControlData() {
  // prepare and send game control data
  control_data.secsRemaining = (uint32)timeCount;
  if (match_type == DEMO) {
    // ball position is not sent during official matches
    control_data.ballXPos = ball_pos[X];
    control_data.ballZPos = ball_pos[Z];
  }
  wb_emitter_send(emitter, &control_data, sizeof(control_data));
}

// initialize devices and data
static void initialize() {
  // necessary to initialize Webots
  wb_robot_init();

  // emitter for sending game control data and receiving 'move' requests
  emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timeCount_STEP);

  // create structs for the robots present in the .wbt file
  int robot_count = 0;
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    WbNodeRef node = wb_supervisor_node_get_from_def(get_robot_def_name(i));
    if (node) {
      robots[i] = robot_new(node);
      robot_count++;
    }
    else
      robots[i] = NULL;
  }

  // to keep track of ball position
  WbNodeRef ball = wb_supervisor_node_get_from_def("BALL");
  if (ball) {
    ball_translation = wb_supervisor_node_get_field(ball, "translation");
    ball_rotation = wb_supervisor_node_get_field(ball, "rotation");
  }

  // initialize game control data
  memset(&control_data, 0, sizeof(control_data));
  memcpy(control_data.header, GAMECONTROLLER_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER) - 1);
  control_data.version = GAMECONTROLLER_STRUCT_VERSION;
  control_data.playersPerTeam = robot_count / 2;
  control_data.state = STATE_INITIAL;
  control_data.secondaryState = STATE2_NORMAL;
  control_data.teams[0].teamNumber = 0;   // changes at half-timeCount
  control_data.teams[1].teamNumber = 1;
  control_data.teams[0].teamColour = TEAM_BLUE;  // does never change
  control_data.teams[1].teamColour = TEAM_RED;

  // eventually read teams names from file
  FILE *file = fopen("teams.txt", "r");
  if (file) {
    fscanf(file, "%[^\n]\n%[^\n]", team_names[0], team_names[1]);
    fclose(file);
  }

  // variable set during official matches
  const char *WEBOTS_ROBOTSTADIUM = getenv("WEBOTS_ROBOTSTADIUM");
  if (WEBOTS_ROBOTSTADIUM) {
    if (strcmp(WEBOTS_ROBOTSTADIUM, "ROUND") == 0) {
      match_type = ROUND;
      printf("Running Robotstadium ROUND match\n");
    }
    else if (strcmp(WEBOTS_ROBOTSTADIUM, "FINAL") == 0) {
      match_type = FINAL;
      printf("Running Robotstadium FINAL match\n");
    }
  }

  if (match_type != DEMO) {
    // start webcam script in background
    system("./webcam.php &");

    // make video: format=480x360, type=MPEG4, quality=75%
    wb_supervisor_start_movie("movie.avi", 480, 360, "MPEG-4/AVC", 100, 1, 0);
  }

  // enable keyboard for manual score control
  wb_robot_keyboard_enable(timeCount_STEP * 10);
}

// compute current ball velocity
static double compute_ball_velocity() {
  
  // ball position at previous timeCount step
  static double x1 = 1.200636;
  static double z1 = -.004987;

  // ball position at current timeCount step
  double x2 = ball_pos[X];
  double z2 = ball_pos[Z];
  

  // compute ball direction
  double dx = x2 - x1;
  double dz = z2 - z1;

  // remember for the next call to this function
  x1 = x2;
  z1 = z2;

  // compute ball velocity
/*  if(emoCount > 0 && sqrt(dx * dx + dz * dz) == 0){
    missFlag = true;
    printf("miss!");
  }*/
  return sqrt(dx * dx + dz * dz);
}

// detect if the ball has hit something (a robot, a goal post, a wall, etc.) during the last timeCount step
// returns: 1 = hit, 0 = no hit
static int ball_has_hit_something() {

  // velocity at previous timeCount step
  static double vel1 = 0.0;

  // current ball velocity
  double vel2 = compute_ball_velocity();

  // a strong acceleration or deceleration correspond to the ball being hit (or hitting something)
  // however some deceleration is normal because the ball slows down due to the rolling and air friction
  // filter noise: if the ball is almost still then forget it
  int hit = vel2 > 0.001 && (vel2 > vel1 * 1.2 || vel2 < vel1 * 0.8);

  // remember for next call
  vel1 = vel2;

  return hit;
}

static void check_keyboard() {

  // allow to modify score manually
  switch (wb_robot_keyboard_get_key()) {
  case WB_ROBOT_KEYBOARD_SHIFT + 'R':
    control_data.teams[TEAM_RED].score--;
    display();
    break;

  case 'R':
    control_data.teams[TEAM_RED].score++;
    display();
    break;

  case WB_ROBOT_KEYBOARD_SHIFT + 'B':
    control_data.teams[TEAM_BLUE].score--;
    display();
    break;

  case 'B':
    control_data.teams[TEAM_BLUE].score++;
    display();
    break;
  }
}


// euler-axes-angle (vrml) to quaternion conversion
static void vrml_to_q(const double v[4], double q[4]) {
  double l = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  if (l > 0.0) {
    q[0] = cos(v[3] / 2);
    l = sin(v[3] / 2) / sqrt(l);
    q[1] = v[0] * l;
    q[2] = v[1] * l;
    q[3] = v[2] * l;
  }
  else {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
}

// quaternion to euler-axes-angle (vrml) conversion
static void q_to_vrml(const double q[4], double v[4]) {
  // if q[0] > 1, acos will return nan
  // if this actually happens we should normalize the quaternion here 
  v[3] = 2.0 * acos(q[0]);
  if (v[3] < 0.0001) {
    // if e[3] close to zero then direction of axis not important
    v[0] = 0.0;
    v[1] = 1.0;
    v[2] = 0.0;
  }
  else {
    // normalise axes
    double n = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    v[0] = q[1] / n;
    v[1] = q[2] / n;
    v[2] = q[3] / n;
  }
}

// quaternion multiplication (combining rotations)
static void q_mult(double qa[4], const double qb[4], const double qc[4]) {
  qa[0] = qb[0]*qc[0] - qb[1]*qc[1] - qb[2]*qc[2] - qb[3]*qc[3];
  qa[1] = qb[0]*qc[1] + qb[1]*qc[0] + qb[2]*qc[3] - qb[3]*qc[2];
  qa[2] = qb[0]*qc[2] + qb[2]*qc[0] + qb[3]*qc[1] - qb[1]*qc[3];
  qa[3] = qb[0]*qc[3] + qb[3]*qc[0] + qb[1]*qc[2] - qb[2]*qc[1];
}

// move robot to a 3d position
static void move_robot_3d(int robot_index, double tx, double ty, double tz, double alpha) {
  if (robots[robot_index]) {
    // set translation
    double trans[3] = { tx, ty, tz };
    wb_supervisor_field_set_sf_vec3f(robots[robot_index]->translation, trans);
    
    // set rotation
    if (robots[robot_index]->model == NAO) {
      // in NAO case we need to add a rotation before
      double rot1[4] = { 1, 0, 0, -1.5708 };
      double rot2[4] = { 0, 1, 0, alpha };
      
      // convert to quaternions
      double q1[4], q2[4], qr[4], rr[4];
      vrml_to_q(rot1, q1);
      vrml_to_q(rot2, q2);
      
      // multiply quaternions
      q_mult(qr, q2, q1);
      
      // convert to VRML
      q_to_vrml(qr, rr);
      
      wb_supervisor_field_set_sf_rotation(robots[robot_index]->rotation, rr);
    }
    else {
      // in DARWIN case we can use this rotation directly
      double rr[4] = { 0, 1, 0, alpha + 1.5708 };
      wb_supervisor_field_set_sf_rotation(robots[robot_index]->rotation, rr);
    }
  }
}

// place robot in upright position, feet on the floor, facing
static void move_robot_2d(int robot_index, double tx, double tz, double alpha) {
  if (robots[robot_index])
    move_robot_3d(robot_index, tx, 0.35, tz, alpha);
}

// move ball to 3d position
static void move_ball_3d(double tx, double ty, double tz) {
  if (ball_translation && ball_rotation) {
    double trans[3] = { tx, ty, tz };
    double rot[4] = { 0, 1, 0, 0 };
    wb_supervisor_field_set_sf_vec3f(ball_translation, trans);
    wb_supervisor_field_set_sf_rotation(ball_rotation, rot);
  }
}

// move ball to 2d position and down on the floor
static void move_ball_2d(double tx, double tz) {
  move_ball_3d(tx, BALL_RADIUS, tz);
}

// handles a "move robot" request received from a robot controller
static void handle_move_robot_request(const char *request) {
  if (match_type != DEMO) {
    fprintf(stderr, "not in DEMO mode: ignoring request: %s\n", request);
    return;
  }

  int robotID, teamID;
  double tx, ty, tz, alpha;
  if (sscanf(request, "move robot %d %d %lf %lf %lf %lf", &robotID, &teamID, &tx, &ty, &tz, &alpha) != 6) {
    fprintf(stderr, "unexpected number of arguments in 'move robot' request: %s\n", request);
    return;
  }

  // move it now!
  printf("executing: %s\n", request);
  int robot_index = robot_get_index(robotID, teamID);
  if (robot_index != -1 && robots[robot_index])
    move_robot_3d(robot_index, tx, ty, tz, alpha);
  else
    fprintf(stderr, "no such robot: %d %d\n", robotID, teamID);
}

// handle a "move ball" request received from a robot controller
static void handle_move_ball_request(const char *request) {
  if (match_type != DEMO) {
    fprintf(stderr, "not in DEMO mode: ignoring request: %s\n", request);
    return;
  }

  double tx, ty, tz;
  if (sscanf(request, "move ball %lf %lf %lf", &tx, &ty, &tz) != 3) {
    fprintf(stderr, "unexpected number of arguments in 'move ball' request: %s\n", request);
    return;
  }

  // move it now!
  printf("executing: %s\n", request);
  move_ball_3d(tx, ty, tz);
}

static void read_incoming_messages() {
  // read while queue not empty
  while (wb_receiver_get_queue_length(receiver) > 0) {
    // I'm only expecting ascii messages
    const char *request = wb_receiver_get_data(receiver);
    if (memcmp(request, "move robot ", 11) == 0)
      handle_move_robot_request(request);
    else if (memcmp(request, "move ball ", 10) == 0)
      handle_move_ball_request(request);
    else
      fprintf(stderr, "received unknown message of %d bytes\n", wb_receiver_get_data_size(receiver));

    wb_receiver_next_packet(receiver);
  }
}

// this is what is done at every timeCount step independently of the game state
static void step() {

  // copy pointer to ball position values
  if (ball_translation)
    ball_pos = wb_supervisor_field_get_sf_vec3f(ball_translation);

  // update robot position pointers
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; i++)
    if (robots[i])
      robots[i]->position = wb_supervisor_field_get_sf_vec3f(robots[i]->translation);

  if (message_steps)
    message_steps--;

  // yield control to simulator
  wb_robot_step(timeCount_STEP);

  // every 480 milliseconds
  if (step_count % 12 == 0)
    sendGameControlData();

  step_count++;

  // did I receive a message ?
  read_incoming_messages();

  // read key pressed
  check_keyboard();
}

// move robots and ball to kick-off position
static void place_to_kickoff() {

  // Manual placement according to the RoboCup SPL Rule Book:
  //   "The kicking-off robot is placed on the center circle, right in front of the penalty mark.
  //   Its feet touch the line, but they are not inside the center circle.
  //   The second field player of the attacking team is placed in front
  //   of one of the goal posts on the height of the penalty mark"

  const double KICK_OFF_X = CIRCLE_DIAMETER / 2 + LINE_WIDTH;
  const double KICK_OFF_Z = (PENALTY_AREA_Z_DIM + FIELD_SIZE_Z) / 4.0;
  const double GOALIE_X = (FIELD_SIZE_X - LINE_WIDTH) / 2.0;
  const double DEFENDER_X = FIELD_SIZE_X / 2 - PENALTY_AREA_X_DIM - LINE_WIDTH;

  // move the two goalies
  //move_robot_2d(get_blue_robot_index(0), -GOALIE_X, 0, 0);
  move_robot_2d(get_blue_robot_index(0), 0, 0, 0);
  move_robot_2d(get_red_robot_index(0), GOALIE_X, 0, M_PI);

  // move other robots
  if (control_data.kickOffTeam == TEAM_RED) {
    //move_robot_2d(get_red_robot_index(1), KICK_OFF_X, 0, M_PI);
    //move_robot_2d(get_red_robot_index(2), PENALTY_BALL_X_POS, PENALTY_AREA_Z_DIM / 2, M_PI);
    //move_robot_2d(get_red_robot_index(3), DEFENDER_X, -PENALTY_AREA_Z_DIM / 2, M_PI);
    move_robot_2d(get_red_robot_index(1), 0, 0, M_PI);
    move_robot_2d(get_red_robot_index(2), 0, 0, M_PI);
    move_robot_2d(get_red_robot_index(3), 0, 0, M_PI);
    
    move_robot_2d(get_blue_robot_index(1), -DEFENDER_X, -GOAL_WIDTH / 4.0, 0);
    move_robot_2d(get_blue_robot_index(2), -DEFENDER_X, -KICK_OFF_Z, 0);
    move_robot_2d(get_blue_robot_index(3), -DEFENDER_X, KICK_OFF_Z, 0);
    
  }
  else {
    //move_robot_2d(get_red_robot_index(1), DEFENDER_X, KICK_OFF_Z, M_PI);
    //move_robot_2d(get_red_robot_index(2), DEFENDER_X, -KICK_OFF_Z, M_PI);
    //move_robot_2d(get_red_robot_index(3), DEFENDER_X, GOAL_WIDTH / 4.0, -M_PI);
    move_robot_2d(get_red_robot_index(1), 0, 0, M_PI);
    move_robot_2d(get_red_robot_index(2), 0, 0, M_PI);
    move_robot_2d(get_red_robot_index(3), 0, 0, M_PI);
    
    move_robot_2d(get_blue_robot_index(1), -DEFENDER_X, PENALTY_AREA_Z_DIM / 2, 0);
    move_robot_2d(get_blue_robot_index(2), -KICK_OFF_X, 0, 0);
    move_robot_2d(get_blue_robot_index(3), -PENALTY_BALL_X_POS, -PENALTY_AREA_Z_DIM / 2, 0);
    
  }

  // reset ball position
  //move_ball_2d(0, 0);
  move_ball_2d(1.2, 0);
}

// run simulation for the specified number of seconds
static void run_seconds(double seconds) {
  int n = 1000.0 * seconds / timeCount_STEP;
  int i;
  for (i = 0; i < n; i++)
    step();
}

static void hold_to_kickoff(double seconds) {
  int n = 1000.0 * seconds / timeCount_STEP;
  int i;
  for (i = 0; i < n; i++) {
    place_to_kickoff();
    step();
  }
}

static void run_initial_state() {
  timeCount = MAX_timeCount;
  control_data.state = STATE_INITIAL;
  display();
  hold_to_kickoff(1);
}

static void run_ready_state() {
  control_data.state = STATE_READY;
  display();
  run_seconds(1);
}

static void run_set_state() {
  control_data.state = STATE_SET;
  display();
  hold_to_kickoff(1);
}

static void run_finished_state() {
  control_data.state = STATE_FINISHED;
  display();
  run_seconds(5);
}

static int is_in_kickoff_team(int robot_index) {
  if (control_data.kickOffTeam == TEAM_RED && robot_is_red(robot_index)) return 1;
  if (control_data.kickOffTeam == TEAM_BLUE && robot_is_blue(robot_index)) return 1;
  return 0;
}

static void getRegEmo(FILE *fp){
  arouNum = (((2 - getAbs(ball_pos[Z])) / 2) - .5) * 2; //wants to get close to midline    
  goalArou = (((.55 - getAbs(robots[4]->position[Z])) / 0.55) - .5) * 2; //wants to stay close to midline
      
  if(defFlag) {
    pleasNum = -0.3;
    goalPleas = 1;
    fprintf(fp, "D,");
  }
  
  if(outFlag) {
    double pleas = ((getAbs(ball_pos[Z]) - 0.7) / 1.3) / 2; // /1.3 to range 0 to 1 then divide by two for range 0 to 0.5
    pleasNum = -0.5 - pleas;
    if(ball_pos[Z] > 0)
      goalPleas = 0.5 + robots[4]->position[Z];
    else
      goalPleas = (robots[4]->position[Z] - 0.5) * -1;
    fprintf(fp, "O,");
  }
  
  if(goalFlag) {
    pleasNum = getAbs(ball_pos[Z] - robots[4]->position[Z]) / 0.5;  //assume goal in middle - normalize
    if(pleasNum > 1) pleasNum = 1;    //limit to 1 if goalie not in middle
      goalPleas = getAbs(ball_pos[Z] - robots[4]->position[Z]) * -1;  //assume goal in middle - normalize;
    if(goalPleas < -1) goalPleas = -1;    //limit to -1
      fprintf(fp, "G,");
  }
  
  if(missFlag) {
    pleasNum = -0.6 - (((getAbs(ball_pos[Z]) - 0.7) / 1.3) / 2.5); // /1.3 to range 0 to 1 then divide by 2.5 for range 0 to 0.4
    goalPleas = .5;
    fprintf(fp, "M,");
  }
  
  if(!goalFlag) {
    if(rPost || lPost){
      pleasNum = -0.4;
      if(rPost){
        goalPleas = robots[4]->position[Z] + 0.5;   //1 next to post, 0 at wrong post
        fprintf(fp, "R,");
      }
      else if(lPost){
        goalPleas = (robots[4]->position[Z] - 0.5) * -1;   //1 next to post, 0 at wrong post
        fprintf(fp, "L,");
      }
    }
  }
}


// detect if a robot has just touched the ball (in the last timeCount step)
// returns: robot index or -1 if there is no such robot
static int detect_ball_touch() {
  FILE *fp;
  fp = fopen("outList.txt", "a");
  if (fp == NULL) {
    printf("Can't open output file");
    exit(1);
  }
    
  if (! ball_has_hit_something())
  {
    //if(emoCount > 300 || defFlag || outFlag || goalFlag || missFlag) {
    if(defFlag || outFlag || goalFlag || missFlag || rPost || lPost) {
      emoCount = 0;
      getRegEmo(fp);
      
      if(ball_pos[X] < 2)
        fprintf(fp, "previous was a mistake\n\n");
      else
        fprintf(fp, "%f,%f,%f,%f\n", pleasNum, arouNum, goalPleas, goalArou); //kickPleas, kickArou, goalPleas, KickArou
      //printf("Record #%d\n", recQuant);
      recQuant++;
      //updateEmo(pleasNum, arouNum);
      
    }
    else if(emoCount != 0){
      emoCount++;
//      printf("EmoCount      %d\n", emoCount);
    }
//    if(outFlag) printf("\nball Out\n");
//    if(goalFlag) printf("\nGoal Scored\n");
//    if(defFlag) printf("\n was defended!\n");
    fclose(fp);
    free(fp);
    return -1;
  }



  // find which robot is the closest to the ball
  double minDist2 = 0.25;  // squared robot proximity radius
  int index = -1;
  int i;

  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    if (robots[i]) {
      double dx = robots[i]->position[X] - ball_pos[X];
      double dz = robots[i]->position[Z] - ball_pos[Z];      
      
      // squared distance between robot and ball
      double dist2 = dx * dx + dz * dz;
      if (dist2 < minDist2) {
        minDist2 = dist2;
        index = i;
      }
    }
  }
  
  //check if hit R post
  double dist2 = getAbs(ball_pos[Z] - 0.7);
  if (dist2 < minDist2) {
    minDist2 = dist2;
    index = -1;
    rPost = true;
//    printf("right post");
  }
  
  //check if hit L post
  dist2 = getAbs(ball_pos[Z] + 0.7);
  if (dist2 < minDist2) {
    minDist2 = dist2;
    index = -1;
    lPost = true;
//    printf("left post");
  }
  
  // print info
  if (index > -1) {
    if (robot_is_red(index)){
//      printf("RED TOUCH   %d\n", index);
      if (emoCount > 0){
//        printf("RED defends!!\n");
        defFlag = true;
        }
    }
    else if (emoCount == 0)
    {
      fprintf(fp, "%f,%f,%f,", robots[index]->position[X], robots[index]->position[Z], kickAngle); //kicker x, z, angle
      fprintf(fp, "%f,", robots[4]->position[Z]);  //goalie z
//      printf("BLUE TOUCH   %d\n", index);
//      printf("print coords\n");
      emoCount = 1;
    }
    
  }
  
  fclose(fp);
  free(fp);
  
  return index;
}

static int is_ball_in_field() {
  return fabs(ball_pos[Z]) <= FIELD_Z_LIMIT && fabs(ball_pos[X]) <= FIELD_X_LIMIT;
}

static int is_ball_in_red_goal() {
  return ball_pos[X] > FIELD_X_LIMIT && ball_pos[X] < FIELD_X_LIMIT + 0.25 && fabs(ball_pos[Z]) < GOAL_WIDTH / 2;
}

static int is_ball_in_blue_goal() {
  return ball_pos[X] < -FIELD_X_LIMIT && ball_pos[X] > -(FIELD_X_LIMIT + 0.25) && fabs(ball_pos[Z]) < GOAL_WIDTH / 2;
}

static int is_ball_in_central_circle() {
  return ball_pos[X] * ball_pos[X] + ball_pos[Z] * ball_pos[Z] < CIRCLE_RADIUS_LIMIT * CIRCLE_RADIUS_LIMIT;
}

static double sign(double value) {
  return value > 0.0 ? 1.0 : -1.0;
}

static void update_kick_off_state() {
  if (kick_off_state == KICK_OFF_INITIAL && ! is_ball_in_central_circle())
    kick_off_state = KICK_OFF_LEFT_CIRCLE;

  int touch_index = detect_ball_touch();
  if (touch_index != -1) {
    last_touch_robot_index = touch_index;

    // "the ball must touch a player from the kick-off team after leaving the center circle
    // before a goal can be scored by the team taking the kick-off"
    if (kick_off_state == KICK_OFF_LEFT_CIRCLE && is_in_kickoff_team(last_touch_robot_index))
      kick_off_state = KICK_OFF_OK;

    if (kick_off_state == KICK_OFF_INITIAL && ! is_in_kickoff_team(last_touch_robot_index))
      kick_off_state = KICK_OFF_OK;
  }
}

// check if throwing the ball in does not collide with a robot.
// If it does collide, change the throw-in location.
static void check_throw_in(double x, double z) {

  // run some steps to see if the ball is moving: that would indicate a collision
  step();
  ball_has_hit_something();
  step();
  ball_has_hit_something(); // because after a throw in, this method return always 1 even if no collision occured
  step();

  while (ball_has_hit_something()) {
    // slope of the line formed by the throw in point and the origin point.
    double slope = z / x;
    z -= sign(z) * 0.1;
    x = z / slope;
    move_ball_2d(x, z);
    check_throw_in(x, z); // recursive call to check the new throw in point.
  }
}

// check if the ball leaves the field and throw ball in if necessary
static void check_ball_out() {

  double throw_in_pos[3];   // x and z throw-in position

  if (fabs(ball_pos[Z]) > FIELD_Z_LIMIT) {  // out at side line
    // printf("ball over side-line: %f %f\n", ball_pos[X], ball_pos[Z]);
    double back;
    if (last_touch_robot_index == -1)  // not sure which team has last touched the ball
      back = 0.0;
    else if (robot_is_red(last_touch_robot_index))
      back = 1.0;  // 1 meter towards red goal
    else
      back = -1.0;  // 1 meter towards blue goal

    throw_in_pos[X] = ball_pos[X] + back;
    throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;

    // in any case the ball cannot be placed off the throw-in line
    if (throw_in_pos[X] > THROW_IN_LINE_X_END)
      throw_in_pos[X] = THROW_IN_LINE_X_END;
    else if (throw_in_pos[X] < -THROW_IN_LINE_X_END)
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
  }
  else if (ball_pos[X] > FIELD_X_LIMIT && ! is_ball_in_red_goal()) {  // out at end line
    // printf("ball over end-line (near red goal): %f %f\n", ball_pos[X], ball_pos[Z]);
    if (last_touch_robot_index == -1) {   // not sure which team has last touched the ball
      throw_in_pos[X] = THROW_IN_LINE_X_END;
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
    else if (robot_is_red(last_touch_robot_index)) { // defensive team
      throw_in_pos[X] = THROW_IN_LINE_X_END;
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
    else { // offensive team
      throw_in_pos[X] = 0.0; // halfway line 
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
  }
  else if (ball_pos[X] < -FIELD_X_LIMIT && ! is_ball_in_blue_goal()) {  // out at end line
    // printf("ball over end-line (near blue goal): %f %f\n", ball_pos[X], ball_pos[Z]);
    if (last_touch_robot_index == -1) {  // not sure which team has last touched the ball
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
    else if (robot_is_blue(last_touch_robot_index)) { // defensive team
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
    else { // offensive team
      throw_in_pos[X] = 0.0; // halfway line 
      throw_in_pos[Z] = sign(ball_pos[Z]) * THROW_IN_LINE_Z;
    }
  }
  else
    return; // ball is not out

  // the ball is out:
  show_message("OUT!");
  outFlag = true;
  kick_off_state = KICK_OFF_OK;

  // let the ball roll for 2 seconds
  run_seconds(2.0);

  // throw the ball in
  move_ball_2d(throw_in_pos[X], throw_in_pos[Z]);
  check_throw_in(throw_in_pos[X], throw_in_pos[Z]);
}

static void run_playing_state() {

  control_data.state = STATE_PLAYING;
  show_message("KICK-OFF!");
  kick_off_state = KICK_OFF_INITIAL;
  last_touch_robot_index = -1;

  while (1) {
    // substract timeCount_STEP to current timeCount
    timeCount -= timeCount_STEP / 1000.0;
    display();

    if (timeCount < 0.0) {
      timeCount = 0.0;
      control_data.state = STATE_FINISHED;
      return;
    }

    update_kick_off_state();

    check_ball_out();

    if (is_ball_in_red_goal()) {  // ball in the red goal

      // a goal cannot be scored directly from a kick-off
      if (control_data.kickOffTeam == TEAM_RED || kick_off_state == KICK_OFF_OK) {
        control_data.teams[TEAM_BLUE].score++;
        show_message("GOAL!");
        goalFlag = true;
      }
      else
        show_message("KICK-OFF SHOT!");

      control_data.state = STATE_READY;
      control_data.kickOffTeam = TEAM_RED;
      return;
    }
    else if (is_ball_in_blue_goal()) {  // ball in the blue goal

      // a goal cannot be scored directly from a kick-off
      if (control_data.kickOffTeam == TEAM_BLUE || kick_off_state == KICK_OFF_OK) {
        control_data.teams[TEAM_RED].score++;
        show_message("GOAL!");
      }
      else
        show_message("KICK-OFF SHOT!");

      control_data.state = STATE_READY;
      control_data.kickOffTeam = TEAM_BLUE;
      return;
    }

    step();
  }
}

static void terminate() {

  if (match_type != DEMO) {
    FILE *file = fopen("scores.txt", "w");
    if (file) {
      if (teams_swapped())
        fprintf(file, "%d\n%d\n", control_data.teams[TEAM_RED].score, control_data.teams[TEAM_BLUE].score);
      else
        fprintf(file, "%d\n%d\n", control_data.teams[TEAM_BLUE].score, control_data.teams[TEAM_RED].score);
      fclose(file);
    }
    else
      fprintf(stderr, "could not write: scores.txt\n");

    // give some timeCount to show scores
    run_seconds(10);

    // freeze webcam
    system("killall webcam.php");

    // terminate movie recording and quit
    wb_supervisor_stop_movie();
    wb_robot_step(0);
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
  }

  while (1) step();  // wait forever
}

static void run_half_timeCount_break() {
  show_message("HALF timeCount BREAK!");
  step();

}

static void run_half_timeCount() {

  // first kick-off of the half is always for the reds
  control_data.kickOffTeam = TEAM_RED;

  run_initial_state();

  do {
    run_ready_state();
    run_set_state();
    run_playing_state();
  }
  while (control_data.state != STATE_FINISHED);

  run_finished_state();
}

// randomize initial position for penalty kick shootout
static double randomize_pos() {
  return (double)rand() / (double)RAND_MAX * 0.01 - 0.005;   // +/- 1 cm
}

// randomize initial angle for penalty kick shootout
static double randomize_angle() {
  return (double)rand() / (double)RAND_MAX * 0.1745 - 0.0873;  // +/- 5 degrees
}

static void run_penalty_kick(double delay, double nbKickAngle, double nbGoalPos) {

  // game control
  timeCount = delay;
  control_data.kickOffTeam = TEAM_BLUE;
  control_data.state = STATE_SET;
  display();
  
  if(ball_pos[X] < 2){
    FILE *fp;
    fp = fopen("outList.txt", "a");
    if (fp == NULL) {
      printf("Can't open output file");
      exit(1);
    }
  
    fprintf(fp, "previous was mistake\n\n");
    fclose(fp);
    free(fp);      
  }
  
  // "The ball is placed on the penalty spot"
  move_ball_2d(PENALTY_BALL_X_POS + randomize_pos(), randomize_pos());

  // attacker and goalie indices during penalties
  int attacker = get_blue_robot_index(ATTACKER);
  int goalie = get_red_robot_index(0);
  
  // move other robots out of the soccer field
  int i, j = 0;
  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    if (robots[i] && i != attacker && i != goalie) {
      // preserve elevation to avoid dropping them or putting them through the floor
      double elevation = wb_supervisor_field_get_sf_vec3f(robots[i]->translation)[Y];
      double out_of_field[3] = { 0.0, elevation, 5.0 + j++ };
      wb_supervisor_field_set_sf_vec3f(robots[i]->translation, out_of_field);
    }
  }
  // "The attacking robot is positioned at the center of the field, facing the ball"
  // "The goal keeper is placed with feet on the goal line and in the centre of the goal"
  //kickAngle = 30;//
  //printf("kicker angle is %f\n", kickAngle);
  kickAngle = nbKickAngle;
  //printf("kicker angle assigned is %f\n", kickAngle);
  //printf("nbKickAngle is %f\n", nbKickAngle);
  
  double kickOri = kickAngle * (PI/180);
  double kickX = PENALTY_BALL_X_POS - cos(kickAngle * (PI/180)) * kickDist + (.0625 * (kickAngle / 90));
  double kickZ = sin(kickAngle * (PI/180)) * kickDist + (.0625 * ((90 - abs(kickAngle)) / (double)90));
  const double ATTACKER_POS[3] = { randomize_pos() + kickX, randomize_pos() + kickZ, kickOri };
  
  double goalieOffset = nbGoalPos;
  const double GOALIE_POS[3] = { PENALTY_GOALIE_X_POS + randomize_pos(), randomize_pos() + goalieOffset, -M_PI + randomize_angle() };
  //printf("goalie pos is %f\n", goalieOffset);

  // hold attacker and goalie for 5 seconds in place during the SET state
  int n;
  for (n = 1000 / timeCount_STEP; n > 0; n--) {
    move_robot_2d(attacker, ATTACKER_POS[0], ATTACKER_POS[1], ATTACKER_POS[2]);
    move_robot_2d(goalie, GOALIE_POS[0], GOALIE_POS[1], GOALIE_POS[2]);
    step();
  }

  // switch to PLAYING state
  control_data.state = STATE_PLAYING;

  do {
    // substract timeCount_STEP to current timeCount
    timeCount -= timeCount_STEP / 1000.0;
    display();

    if (timeCount < 0.0) {
      timeCount = 0.0;
      show_message("timeCount OUT!");
      missFlag = true;
      return;
    }

    //int robot_index = detect_ball_touch();
    detect_ball_touch();

    // "If the goal keeper touches the ball outside the penalty area then a goal will be awarded to the attacking team"
   /* if (robot_index == goalie && ball_completely_outside_penalty_area()) {
      control_data.teams[TEAM_RED].score++;
      show_message("ILLEGAL GOALIE ACTION!");
      return;
    }
    // "If the attacker touched the ball inside the penalty area then the penalty shot is deemed unsuccessful"
    else if (robot_index == attacker && ball_completely_inside_penalty_area()) {
      show_message("ILLEGAL ATTACKER ACTION!");
      return;
    } */

    step();
  }
  while (is_ball_in_field() && !defFlag);

  if (is_ball_in_red_goal()) {
    control_data.teams[TEAM_RED].score++;
    //show_message("GOAL!");
    goalFlag = true;
    detect_ball_touch();
  }
  else{
    if(defFlag){
      //show_message("DEFENDED!");
      detect_ball_touch();
    }
    else {
      //show_message("MISSED!");
      outFlag = true;
      detect_ball_touch();
    }
  }
}

static void run_penalty_kick_shootout() {

  //show_message("PENALTY KICK SHOOT-OUT!");
  step();
  // inform robots of the penalty kick shootout
  control_data.secondaryState = STATE2_PENALTYSHOOT;
  
  //curEvent[5]   angle, goalie, pleas, arou, id
  double stressLvl = 0.0;
  // five penalty shots per team
  int i = 0, j = 0;
  while(i == 0) {
    FILE* fAngle = fopen("nbOutTest.txt", "r");
    FILE *goalOut = fopen("wbOutTest.txt", "w");
    if (fAngle == NULL || goalOut == NULL) {
      printf("Can't open input and/or output file");
      exit(1);
    }
    if(j==0){
      pleasNum = curEvent[2] = -0.6;
      arouNum = curEvent[3] = -0.6;  //adds 0.1 below
      j = 1;
    }
    else {
      /*if (curEvent[2] < pleasNum) {
        curEvent[2] += 0.1;
        if (curEvent[2] > 1) {
          curEvent[2] = 1.0;
        }
      }
      else {
        curEvent[2] -= 0.1;
        if (curEvent[2] < -1) {
          curEvent[2] = 1.0;
        }
      }
      if (curEvent[3] < arouNum) {
        curEvent[3] += 0.1;
        if (curEvent[3] > 1) {
          curEvent[3] = 1.0;
        }
      }
      else {
        curEvent[3] -= 0.1;
        if (curEvent[3] < -1) {
          curEvent[3] = 1.0;
        }
      } */
      printf("\ncurPleas - %f      pleas - %f", curEvent[2], pleasNum);
      printf("\ncurArou - %f      arou - %f\n", curEvent[3], arouNum);
      if(stressLvl < 3){
        if(getAbs(curEvent[2] - pleasNum) > 0.3) {
          stressLvl += 0.3;
        }
        else if(getAbs(curEvent[3] - arouNum) > 0.3) {
          stressLvl += 0.3;
        }
        else if(getAbs(curEvent[2] - pleasNum) < 0.1) {
          stressLvl -= 0.1;
        }
        else if(getAbs(curEvent[3] - arouNum) < 0.1) {
          stressLvl -= 0.1;
        }
      }
      if(stressLvl < 0) stressLvl = 0;
      if(stressLvl == 20) {
        curEvent[2] = 1 - ((rand() % 21) * 0.1);
        if(curEvent[2] > 0)
          curEvent[3] = 1 - ((rand() % 10) * 0.1);
        else
          curEvent[3] = -1 + ((rand() % 10) * 0.1);
        stressLvl = 0;
      }
      else if(stressLvl >= 2) {
        if(curEvent[2] > 0) 
          curEvent[2] = -1 + ((rand() % 10) * 0.1);
        else
          curEvent[2] = 1 - ((rand() % 10) * 0.1);
        if(curEvent[2] > 0) 
          curEvent[3] = -1 + ((rand() % 10) * 0.1);
        else
          curEvent[3] = 1 - ((rand() % 10) * 0.1);  
        //stressLvl = 20;
        stressLvl = 0;
        printf("\n\n*****STRESSED******\n\n");
      }
      else {
        //curEvent[2] = (pleasNum + curEvent[2]) / 2;
        //curEvent[3] = (arouNum + curEvent[3]) / 2;
        if (curEvent[2] < pleasNum) {
          curEvent[2] += 0.1;
          if (curEvent[2] > 1) {
            curEvent[2] = 1.0;
          }
        }
        else {
          curEvent[2] -= 0.1;
          if (curEvent[2] < -1) {
            curEvent[2] = -1.0;
          }
        }
        if (curEvent[3] < arouNum) {
          curEvent[3] += 0.1;
          if (curEvent[3] > 1) {
            curEvent[3] = 1.0;
          }
        }
        else {
          curEvent[3] -= 0.1;
          if (curEvent[3] < -1) {
            curEvent[3] = -1.0;
          }
        }
      }  //hey
      //curEvent[1] = 0.5 - ((rand() % 21) * 0.05);  //randomize goalie pos
    }
    printf("\nStress - %f\n", stressLvl);
    //printf("\n\ncurPleas - %f\n\n", curEvent[2]);
    //printf("\n\ncurArou - %f\n\n", curEvent[3]);
    
    //curEvent[2] += (((rand() % 3) * 0.1 - 0.1));  //randomize add to kickpleas;
    //if(curEvent[2] > 1) curEvent[2] = 1;
    //if(curEvent[2] < -1) curEvent[2] = -1;
    //curEvent[3] = 1 - ((rand() % 21) * 0.1);  //randomize kickArou;
    curEvent[1] = 0.5 - ((rand() % 21) * 0.05);  //randomize goalie pos
    fprintf(goalOut, "%f,%f,%f,%d,", curEvent[1], curEvent[2], curEvent[3], curEventID);
    if(goalFlag) fprintf(goalOut, "G");
    else if(defFlag) fprintf(goalOut, "D");
    else if(outFlag) fprintf(goalOut, "O");
    else if(rPost) fprintf(goalOut, "R");
    else if(lPost) fprintf(goalOut, "L");
    //pleasNum, arouNum
    else if(ball_pos[X] < 2) fprintf(goalOut, "XX");
    else fprintf(goalOut, "M");
    goalFlag = defFlag = outFlag = missFlag = rPost = lPost = false;
    printf("\nprinted\n");
    fclose(goalOut);
  
    int j = 0;
    float n;
  
    while(curEvent[4] != curEventID) {
      //printf("\nscanning\n");
//      printf("curEvent - %f   event ID - %d\n", curEvent[4], curEventID);
      while((fscanf(fAngle, "%f,", &n) > 0)){
        if (j == 0) curEvent[0] = n;
        if (j == 1) curEvent[4] = n;
        j++;
//        printf("\nscanned\n");
      }
      rewind(fAngle);
      j = 0;
    }
    curEventID++;
    printf("kicker angle %f\nEvent ID %f", curEvent[0], curEvent[4]);
  
    fclose(fAngle);
  
    run_penalty_kick(60, curEvent[0], curEvent[1]);

  }
  printf("Game Over!");
}

int main(int argc, const char *argv[]) {

  // init devices and data structures
  initialize();
  /* initialize random seed: */
  srand (time(NULL));

  FILE *fp;
  fp = fopen("outList.txt", "a");
  if (fp == NULL) {
    printf("Can't open output file");
    exit(1);
  }  
  fprintf(fp, "kickX, kickY, kickAngle, goalZ, outcome, kickPleas, kickArou, goalPleas, goalArou\n");
  fprintf(fp, "Outcomes: D - defended, O - out, G - goal, M - missed, R - rightPost,L - leftPost\n\n");
  fclose(fp);
  //free(fp);

  run_penalty_kick_shootout();
  
  // terminate movie, write scores.txt, etc.
  terminate();

  return 0; // never reached
}

