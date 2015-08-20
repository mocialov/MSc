/*
 * File:         contact_points_physics.c
 * Date:         November 6, 2009
 * Description:  Example of use of a physics plugin to get information on
 *               the contact points and ground reaction forces of an object
 *               that collides with the floor
 * Author:       Yvan Bourquin
 *
 * Copyright (c) 2009 Cyberbotics - www.cyberbotics.com
 */

#include <plugins/physics.h>

#define MAX_CONTACTS 10

static pthread_mutex_t mutex;

// plugin variables
static dGeomID box_geom = NULL;
static dBodyID box_body = NULL;
static dJointID contact_joints[MAX_CONTACTS];
static dGeomID floor_geom = NULL;
static dContact contacts[MAX_CONTACTS];
static int nContacts = 0;
static dJointFeedback feedbacks[MAX_CONTACTS];

// plugin function called by Webots at the beginning of the simulation
void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j) {
  pthread_mutex_init(&mutex, NULL); // needed to run with multi-threaded version of ODE

  // get ODE handles to .wbt objects
  floor_geom = dWebotsGetGeomFromDEF("FLOOR");
  box_geom = dWebotsGetGeomFromDEF("OBJECT");
  
  // get box's body (the floor does not have a body)
  box_body = dGeomGetBody(box_geom);
}

// plugin function called by Webots for every WorldInfo.basicTimeStep
void webots_physics_step() {
  nContacts = 0;
}

// it is not really necessary but it's fun to draw the contact points
void webots_physics_draw(int pass, const char *view) {
  if (pass != 1)
    return;

  // change OpenGL state
  glDisable(GL_LIGHTING);    // not necessary
  glLineWidth(10);           // use a thick line
  glDisable(GL_DEPTH_TEST);  // draw in front of Webots graphics

  int i;
  for (i = 0; i < nContacts; i++) {
    dReal *p = contacts[i].geom.pos;
    dReal *n = contacts[i].geom.normal;
    dReal d = contacts[i].geom.depth * 30;
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);  // draw in red
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(p[0] + n[0] * d, p[1] + n[1] * d, p[2] + n[2] * d);
    glEnd();
  }
}

// This function is implemented to overide Webots collision detection.
// It returns 1 if a specific collision is handled, and 0 otherwise. 
int webots_physics_collide(dGeomID g1, dGeomID g2) {
  // check if this collision involves the objects which interest us
  if ((dAreGeomsSame(g1, box_geom) && dAreGeomsSame(g2, floor_geom)) || (dAreGeomsSame(g2, box_geom) && dAreGeomsSame(g1, floor_geom))) {
    dBodyID body = dGeomGetBody(g1);
    if (body==NULL) body = dGeomGetBody(g2);
    if (body==NULL) return 0;
    dWorldID world = dBodyGetWorld(body);
    dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
    // see how many collision points there are between theses objects
    nContacts = dCollide(g1, g2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));
    int i;
    for (i = 0; i < nContacts; i++) {
      // custom parameters for creating the contact joint
      // remove or tune these contact parameters to suit your needs
      contacts[i].surface.mode = dContactBounce | dContactSoftCFM | dContactApprox1;
      contacts[i].surface.mu = 1.5;
      contacts[i].surface.bounce = 0.5;
      contacts[i].surface.bounce_vel = 0.01;
      contacts[i].surface.soft_cfm = 0.001;
    
      // create a contact joint that will prevent the two bodies from intersecting
      // note that contact joints are added to the contact_joint_group
      pthread_mutex_lock(&mutex);
      contact_joints[i] = dJointCreateContact(world, contact_joint_group, &contacts[i]);
      
      // attach joint between the body and the static environment (0)
      dJointAttach(contact_joints[i], box_body, 0);
        
      // attach feedback structure to measure the force on the contact joint
      dJointSetFeedback(contact_joints[i], &feedbacks[i]);
      pthread_mutex_unlock(&mutex);
    }

    return 1;  // collision was handled above
  }

  return 0;  // collision must be handled by webots
}

// convenience function to print a 3d vector
static void print_vec3(const char *msg, const dVector3 v) {
  dWebotsConsolePrintf("%s: %g %g %g\n", msg, v[0], v[1], v[2]);
}

// this function is called by Webots after dWorldStep()
void webots_physics_step_end() {
  if (nContacts == 0) {
    dWebotsConsolePrintf("no contact\n");
    return;
  }

  int i;
  for (i = 0; i < nContacts; i++) {
    // printf force and torque that each contact joint
    // applies on the box's body
    dWebotsConsolePrintf("contact: %d:\n", i);
    print_vec3("f1", feedbacks[i].f1);
    print_vec3("t1", feedbacks[i].t1);
    print_vec3("f2", feedbacks[i].f2);
    print_vec3("t2", feedbacks[i].t2);
    dWebotsConsolePrintf("\n");
  }  
}

// this function is called by Webots to cleanup memory before unloading the physics plugin
void webots_physics_cleanup() {
  pthread_mutex_destroy(&mutex);
}
