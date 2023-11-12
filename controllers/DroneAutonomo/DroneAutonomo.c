#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/supervisor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/accelerometer.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define PI 3.14159265358979323846
int point = 0;

struct Coordenadas {
	double x;
	double y;
};

struct PIDController {
	double Kp;
	double Ki;
	double Kd;
	double prev_error;
	double integral;
	double setpoint;
	int id;
};

struct PIDController createPIDController(double Kp, double Ki, double Kd, int id) {
	struct PIDController controller;
	controller.Kp = Kp;
	controller.Ki = Ki;
	controller.Kd = Kd;
	controller.prev_error = 0.0;
	controller.integral = 0.0;
	controller.setpoint = 0.0;
	controller.id = id;
	return controller;
}

double PID_Update(struct PIDController* pid, double process_variable) {
	double error = pid->setpoint - process_variable;
	double output;

	//printf("%f\n", error);
	double time = 0.008;
	double P = pid->Kp * error;
	pid->integral += error;
	double I = pid->Ki * pid->integral;
	double D = pid->Kd * (error - pid->prev_error);
	output = P + I + D;

	printf("P: %f, I: %f, D: %f, output: %f, setpoint: %f, error: %f, ID: %d\n", P, I, D, output, pid->setpoint, error,pid->id);
	pid->prev_error = error;

	return output;
}

double degreeToRadian(double degree) {
	return degree * PI / 180.0;
}

int main(int argc, char** argv) {
	wb_robot_init();
	int timestep = (int)wb_robot_get_basic_time_step();
	WbDeviceTag camera = wb_robot_get_device("camera");
	wb_camera_enable(camera, timestep);
	WbDeviceTag front_left_led = wb_robot_get_device("front left led");
	WbDeviceTag front_right_led = wb_robot_get_device("front right led");
	WbDeviceTag imu = wb_robot_get_device("inertial unit");
	wb_inertial_unit_enable(imu, timestep);
	WbDeviceTag gps = wb_robot_get_device("gps");
	wb_gps_enable(gps, timestep);
	WbDeviceTag compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, timestep);
	WbDeviceTag gyro = wb_robot_get_device("gyro");
	wb_gyro_enable(gyro, timestep);
	wb_keyboard_enable(timestep);


	WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
	WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
	WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");

	WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
	WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
	WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
	WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
	WbDeviceTag motors[4] = { front_left_motor, front_right_motor, rear_left_motor, rear_right_motor };
	int m;
	for (m = 0; m < 4; ++m) {
		wb_motor_set_position(motors[m], INFINITY);
		wb_motor_set_velocity(motors[m], 1.0);
	}

	while (wb_robot_step(timestep) != -1) {
		if (wb_robot_get_time() > 1.0)
			break;
	}

	// Constants, empirically found.
	const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
	const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
	const double k_vertical_p = 3.0;        // P constant of the vertical PID.
	const double k_roll_p = 50.0;           // P constant of the roll PID.
	const double k_pitch_p = 30.0;          // P constant of the pitch PID.

	// Variables.
	double target_altitude = 1.0;
	double startAltitude = 0;

	struct PIDController pitchPID = createPIDController(100, 100, 5, 1);
	struct PIDController rollPID = createPIDController(100, 100, 5, 2);
	struct PIDController yawPID = createPIDController(10, 0.09, 1, 3);
	struct PIDController goPID = createPIDController(0.5, 0, 1, 4); 

	struct Coordenadas locationPoints[5] = {
	{5, 5},
	{5, -5},
	{-5, -5},
	{-5, 5},
	{0, 0}
	};


	bool firstRun = true;
	bool stable = false;
	bool yawStable = false;
	int stablee = 0;
	double anteriorX = 0;
	double velocidadeX = 0;
	double anteriorY = 0;
	double velocidadeY = 0;
	double initialX = 0;
	double initialY = 0;


	// Main loop
	while (wb_robot_step(timestep) != -1) {
		const double time = wb_robot_get_time();  // in seconds.

		const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
		const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
		const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
		const double X = wb_gps_get_values(gps)[0];
		const double Y = wb_gps_get_values(gps)[1];
		const double Z = wb_gps_get_values(gps)[2];
		const double roll_velocity = wb_gyro_get_values(gyro)[0];
		const double pitch_velocity = wb_gyro_get_values(gyro)[1];
		const double yaw_velocity = wb_gyro_get_values(gyro)[2];

		velocidadeX = (X - anteriorX) / timestep;
		anteriorX = X;

		velocidadeY = (Y - anteriorY) / timestep;
		anteriorY = Y;


		double angle = yaw;

		if (angle < 0)
		{
			angle = angle + 2 * PI;
		}

		double degree = angle * 180 / PI; 
		double pitchAux = 0.0;
		double rollAux = 0.0;
		double aux;

		if (degree >= 0 && degree < 90) { 
			aux = 90 - degree; 
			rollAux = velocidadeX * cos(degreeToRadian(aux)) + velocidadeY * cos(degreeToRadian(90 + 2 * aux));
			pitchAux = velocidadeX * sin(degreeToRadian(aux)) + velocidadeY * sin(degreeToRadian(90 + 2 * aux));
		}
		else if (degree >= 90 && degree < 180) { 
			aux = degree - 90; 
			rollAux = velocidadeX * cos(degreeToRadian(aux)) + velocidadeY * cos(degreeToRadian(degree - 2 * aux));
			pitchAux = velocidadeX * sin(degreeToRadian(aux)) + velocidadeY * sin(degreeToRadian(degree - 2 * aux)); 
		}
		else if (degree >= 180 && degree < 270) { 
			aux = 90 - degree; 
			rollAux = velocidadeX * cos(degreeToRadian(aux)) + velocidadeY * cos(degreeToRadian(aux + 90));
			pitchAux = velocidadeX * sin(degreeToRadian(aux)) + velocidadeY * sin(degreeToRadian(aux + 90));
		}
		else if (degree >= 270 && degree <= 360) { 
			aux = 360 - degree + 90; 
			rollAux = velocidadeX * cos(degreeToRadian(aux)) + velocidadeY * cos(degreeToRadian(aux + 90));
			pitchAux = velocidadeX * sin(degreeToRadian(aux)) + velocidadeY * sin(degreeToRadian(aux + 90));
		}

		double roll_disturbance = 0.0;
		double pitch_disturbance = 0.0;
		double yaw_disturbance = 0.0;

		// INICIO
		double deltaX = locationPoints[point].x - X;
		double deltaY = locationPoints[point].y - Y;
		double distance = sqrt(deltaX * deltaX + deltaY * deltaY); 
		double destinationAngle = atan2(deltaY, deltaX);

		/*yawPID.setpoint = destinationAngle;*/
		double erroAngle = destinationAngle - yaw;

		if (firstRun == true)
		{
			startAltitude = Z;
			firstRun = false;
			yawPID.setpoint = destinationAngle; 
			initialX = X;
			initialY = Y;
		}
		
		if (Z > startAltitude + 0.15)
		{
			if ((pitchAux < 0.0009 && pitchAux > -0.0009) && 
				(rollAux < 0.0009 && rollAux > -0.0009) &&
				(yaw_velocity < 0.0009 && yaw_velocity > -0.0009) && 
				(erroAngle < 0.09 && erroAngle > -0.09))
			{
				yawStable = true;
			}
			roll_disturbance = -PID_Update(&rollPID, rollAux); 
			pitch_disturbance = -PID_Update(&pitchPID, pitchAux);  
			yaw_disturbance = PID_Update(&yawPID, yaw);

			if (yawStable == true) {
				pitch_disturbance += CLAMP(PID_Update(&goPID, distance), -2, 2); 
			}
		}
	
		//printf("X %f, Y %f, pitchDisturbance: %f, rollDisturbance: %f, yawDisturbance: %f\n", X, Y, pitch_disturbance, roll_disturbance, yaw_disturbance);

		const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
		const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
		const double yaw_input = yaw_disturbance;
		const double clamped_difference_altitude = CLAMP(target_altitude - Z + k_vertical_offset, -1.0, 1.0);
		const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

		const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
		const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
		const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
		const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
		wb_motor_set_velocity(front_left_motor, front_left_motor_input);
		wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
		wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
		wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
	};

	wb_robot_cleanup();

	return EXIT_SUCCESS;
}
