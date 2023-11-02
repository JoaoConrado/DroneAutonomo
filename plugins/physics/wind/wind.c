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
	/*dVector3 f;
	f[0] = 0.0;
	f[1] = 0.0;
	f[2] = 0.0;

	dBodyAddForce(gRobotBody, f[0], f[1], f[2]);*/
	dWorldSetGravity(world, 0.5 ,0, -9.81);
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
	return 0;
}

void webots_physics_cleanup() {
	pthread_mutex_destroy(&mutex);
}