#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
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
#define TIMESTAMP 8

int errorCount = 0;
int cemPontos = 0;

typedef struct {
	double Kp;
	double Ki;
	double Kd;
	double prev_error;
	double integral;
	double setpoint;
} PIDController;

void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double setpoint) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->prev_error = 0.0;
	pid->integral = 0.0;
	pid->setpoint = setpoint;
}

double PID_Update(PIDController* pid, double process_variable) {
	double error = pid->setpoint - process_variable;
	double output;

	double time = 0.008;
	double P = pid->Kp * error;
	pid->integral += error;
	double I = pid->Ki * pid->integral;
	double D = pid->Kd * (error - pid->prev_error);
	output = P + I + D;

	pid->prev_error = error;

	return output;
}



struct Coordenadas {
	int x;
	int y;
};

void calcularDiferenca(double x_atual, double y_atual, double angle, double x_desejado, double y_desejado, double* pitch, double* roll, double* yaw, int* rotating, double* yawVelocity, double* erros) {
	double deltaX = x_desejado - x_atual;
	double deltaY = y_desejado - y_atual;

	double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

	double destinationAngle = atan2(deltaY, deltaX);
	double erroAngle = angle - destinationAngle;

	if (erroAngle < -PI || erroAngle > PI)
	{
		erroAngle = destinationAngle + angle;
	}

	double kpYaw = -10;
	double kiYaw = 0.5;
	double kdYaw = 0.0;

	*yaw = CLAMP(kpYaw * erroAngle + kiYaw * erroAngle + kdYaw * erroAngle, -3, 3);

	double media = 0;

	if (*rotating == 1)
	{
		erros[errorCount] = erroAngle;
		errorCount++;

		if (errorCount == 1000)
		{
			errorCount = 0;
			cemPontos = 1;
		}
		if (cemPontos == 0)
		{
			for (int i = 0; i < errorCount; i++)
			{
				media += erros[i];
			}
			media /= errorCount;
		}
		else
		{
			for (int i = 0; i < 1000; i++)
			{
				media += erros[i];
			}
			media /= 1000;
		}

		if (!((media < -0.0005 || media > 0.0005) && (*yawVelocity < -0.00005 || *yawVelocity > 0.00005)))
		{
			*rotating = 0;
		}
	}

	if (*rotating == 0)
	{
		double kpPitch = 0.3;
		double kiPitch = -0.005;
		double kdPitch = 0.0;

		*pitch = -CLAMP(kpPitch * distance + kiPitch * distance + kdPitch * distance, -3, 3);


		/*	double kpRoll = 8.0;
			double kiRoll = 0.2;
			double kdRoll = 0.0;*/

			//double deltaY = (x_atual - y_atual) * sin(PI / 2 - angle);

			//*roll = CLAMP(kpRoll * deltaY + kiRoll * deltaY + kdRoll * deltaY, -2, 2);
	}
	printf("yaw: %f angle: %f, destinationAngle: %f erroAngle: %f media: %f errorCount: %d\n", *yaw, angle, destinationAngle, erroAngle, media, errorCount);
}


int main(int argc, char** argv) {
	wb_robot_init();
	int timestep = (int)wb_robot_get_basic_time_step();

	// Get and enable devices.
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
	WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

	// Get propeller motors and set them to velocity mode.
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


	// Constants, empirically found.
	const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
	const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
	const double k_vertical_p = 3.0;        // P constant of the vertical PID.
	const double k_roll_p = 50.0;           // P constant of the roll PID.
	const double k_pitch_p = 30.0;          // P constant of the pitch PID.

	// Variables.
	double target_altitude = 1.0;  // The target altitude. Can be changed by the user.


	// Variaveis Customizadas
	struct Coordenadas arrayDeCoordenadas[5] = {
	{5, 5},
	{5, -5},
	{-5, -5},
	{-5, 5},
	{0, 0}
	};

	size_t tamanhoPontos = sizeof(arrayDeCoordenadas) / sizeof(arrayDeCoordenadas[0]);

	int pontoAtual = 0;
	int rotating = 1;
	double erros[1000];
	bool launching = true;

	for (int i = 0; i < 1000; i++) {
		erros[i] = 0.0;
	}

	PIDController pitchPID, rollPID, yawPID;

	PID_Init(&pitchPID, 5, 0.005, 2, 0);
	PID_Init(&rollPID, 5, 0.005, 5, 0);
	PID_Init(&yawPID, 20, 0.005, 15, 0);


	// Main loop
	while (wb_robot_step(timestep) != -1) {

		const double time = wb_robot_get_time();  // in seconds.

		// Retrieve robot position using the sensors.
		const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
		const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
		const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
		const double X = wb_gps_get_values(gps)[0];
		const double Y = wb_gps_get_values(gps)[1];
		const double Z = wb_gps_get_values(gps)[2];
		const double roll_velocity = wb_gyro_get_values(gyro)[0];
		const double pitch_velocity = wb_gyro_get_values(gyro)[1];
		const double yaw_velocity = wb_gyro_get_values(gyro)[2];

		// Transform the keyboard input to disturbances on the stabilization algorithm.
		double roll_disturbance = 0.0;
		double pitch_disturbance = 0.0;
		double yaw_disturbance = 0.0;

		double deltaX = arrayDeCoordenadas[pontoAtual].x - X;
		double deltaY = arrayDeCoordenadas[pontoAtual].y - Y;

		double destinationAngle = atan2(deltaY, deltaX);
		yawPID.setpoint = 3.14;

		double normalizedYaw = 0;
		if (yaw < 0)
		{
			normalizedYaw = yaw + 2 * PI;
		}
		else
		{
			normalizedYaw = yaw;
		}

		pitch_disturbance = CLAMP(PID_Update(&pitchPID, X), -10, 10);
		roll_disturbance = -CLAMP(PID_Update(&rollPID, Y), -10, 10);
		yaw_disturbance = CLAMP(PID_Update(&yawPID, normalizedYaw), -10, 10);
		//printf("%f\n", yawPID.prev_error);

		printf("x: %f y: %f z: %f yaw: %f pitchDisturbance: %f rollDisturbance: %f yawDisturbance: %f ponto: %d\n", X, Y, Z, yaw, pitch_disturbance, roll_disturbance, yaw_disturbance, pontoAtual);

		//printf("pitch %f roll %f yawpid %f\n", pitchPID.integral, rollPID.integral, yawPID.integral);
		// Compute the roll, pitch, yaw and vertical inputs.
		const double roll_input = k_roll_p * roll + roll_velocity + roll_disturbance;
		const double pitch_input = k_pitch_p * pitch + pitch_velocity + pitch_disturbance;
		const double yaw_input = yaw_disturbance;
		const double clamped_difference_altitude = CLAMP(target_altitude - Z + k_vertical_offset, -1.0, 1.0);
		const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

		// Actuate the motors taking into consideration all the computed inputs.
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