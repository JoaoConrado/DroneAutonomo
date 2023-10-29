#include <ode/ode.h>
#include <plugins/physics.h>

static pthread_mutex_t mutex;
static dBodyID gRobotBody = NULL;
double f[3]= {0.0,0.05,0};

void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);
  
  gRobotBody = dWebotsGetBodyFromDEF("DRONE");
}

void webots_physics_step() {
  dBodyAddForce(gRobotBody, f[0], 10, f[2]);
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;
}

void webots_physics_cleanup() {
  pthread_mutex_destroy(&mutex);
}
