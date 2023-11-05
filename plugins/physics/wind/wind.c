#include <ode/ode.h>
#include <plugins/physics.h>

static pthread_mutex_t mutex;
static dBodyID gRobotBody = NULL;
static dWorldID world = NULL;

void webots_physics_init() {
	pthread_mutex_init(&mutex, NULL);
	gRobotBody = dWebotsGetBodyFromDEF("DRONE");
	world = dBodyGetWorld(gRobotBody);
}

void webots_physics_step() {
	dWorldSetGravity(world, 1 ,0, -9.81);
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
	return 0;
}

void webots_physics_cleanup() {
	pthread_mutex_destroy(&mutex);
}