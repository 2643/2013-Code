#include "WPILib.h"
#include "Math.h"
#include <PIDcontroller.h>
#define SHOOTER_COUNT 1
#define BOTDRIVE 1
/*
 * Diary:
 * 2/2/13 We made this diary.
 2/9/13 We decided that this diary may end in a few days.
 2/11/13 We ended this diary because we're too lazy to write.
 2/12/13 Added 'dumb code' for Autonomous
 also improved tank and arcade drive code.
 2-18-13 autonomous code...............  :D
 2-19-13 Atonomous code is smarter.....  :D
 13/3/16 fixed some problems from Madera
 
 TODO
 - DSLCD rate experiment
 - smart auton
 - vision

 */
#define YEAR_2013 1
#define CAMERA 0
#define DUMB_DRIVE_CODE 0
#define INTELLIGENT_AUTONOMOUS 0
#define SHOOTER_STOP 1
#define JAGUAR_SWITCH 1
#define TIMER_RESET 0
//PWMs
#if YEAR_2013
const int left_drive_motor_A_PWM = 5;//1
const int left_drive_motor_B_PWM = 6;//2
const int right_drive_motor_A_PWM = 1;//5
const int right_drive_motor_B_PWM = 2;//6
const int shooter_front_motor = 4;//3
const int shooter_back_motor = 3; //4
const int climbing_motor_PWM = 7;
#else
const int left_drive_motor_A_PWM = 3;
const int right_drive_motor_A_PWM = 1;
const int shooter_front_motor = 4;
const int shooter_back_motor = 6;
#endif

// Joysticks
const int operator_joystick = 3;
const int right_stick = 2;
const int left_stick = 1;

// Joystick Buttons

// Driver Stick 1ls
const int arcade_button = 10;
const int tank_button = 11;
// Driver Stick 2
const int climber_off_button = 1;
const int climber_hold_down = 2;
const int climber_hold_up = 3;
const int climber_send_bottom = 10;
const int climber_send_top = 11;
// Operator Stick
const int shooter_piston_button = 1;
const int tilt_up_button = 5;
const int tilt_down_button = 4;
const int front_position_button = 3;
const int back_position_button_1 = 2;
const int back_position_button_2 = 6;
const int back_position_button_3 = 7;
const int back_position_button_4 = 11;
const int back_position_button_5 = 10;
const int dumper_button_A = 8;
const int dumper_button_B = 9;

//Speeds
const float front_position_RPS = 55;
const float back_position_RPS_1 = 46;// was 36.5 at  34 degrees
const float back_position_RPS_2 = 36.5;
const float back_position_RPS_3 = 38.0;
const float back_position_RPS_4 = 36.8;
const float back_position_RPS_5 = 36.9;
const float dumper_RPS = 20;

//Limit Switches -DI
const int top_claw_limit_switch_port = 6;
const int bottom_claw_limit_switch_port = 7;//7



//Encoders DI
const int shooter_motor_front_encoder_A_port = 1;//not changed
const int shooter_motor_front_encoder_B_port = 2;
#if YEAR_2013
const int shooter_motor_back_encoder_A_port = 8;//3 -> changed for testing
const int shooter_motor_back_encoder_B_port = 9;//4
#else
const int shooter_motor_back_encoder_A_port = 7;
const int shooter_motor_back_encoder_B_port = 8;
#endif

enum STATES
{
	stabilizing, unstable, retracting, retract, fire
};

enum CLIMBER
{
	sendup, go_up, go_down, stop
};
enum CLIMBER smart_climber_state = sendup;
enum STATES smart_autonomous_state = unstable;

//Solenoids
const int SHOOTER_ANGLE_SOLENOID_1 = 3;
const int SHOOTER_ANGLE_SOLENOID_2 = 4;
#if YEAR_2013
const int shooter_fire_piston_solenoid_A = 1;
const int shooter_fire_piston_solenoid_B = 2;
#else
const int shooter_fire_piston_solenoid_A = 4;
const int shooter_fire_piston_solenoid_B = 5;
#endif
//Variables
const double shooter_piston_delay = 1.0;
const float first_pterm = .4;
const float iterm = 0.0005;
const float dterm = 0.0;
//for auton fire- ask tony 4 more info
//fires
const int first_fire = 5;
const int second_fire = 9;
const int third_fire =13;
//retracts
const int first_retract = 6;
const int second_retract = 11;
const int third_retract = 14;

const float pidtime = 0.2;//0.03
const int Array_size = 32;

const int PRESSURE_SWITCH = 14;
const int compressor_enable = 2;

#if CAMERA
const char *AxisCameraIpAddr = "10.26.43.11";
#endif

const float shooter_motor_speed = 1.0;//this is for testing purposes, should be 1.0
const float piston_wait = 1.0;
const float fire_wait = 4.0;
const float up_to_speed_wait = 5.0;
const float Adjustment_Speed = 0.5;

//Arrays
const float distance_lookup_array[Array_size] =
{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0,
		14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0,
		26.0, 27.0, 28.0, 29.0, 30.0, 31.0 };// placeholder values

class RobotDemo: public SimpleRobot
{
#if BOTDRIVE
	RobotDrive *drive; // robot drive base object
#endif
	DriverStation *ds; // driver station object for getting selections
	//Joysticks
	Joystick *drive_stick_sec;
	Joystick *drive_stick_prim;
	Joystick *operator_stick;

	//Motors
#if JAGUAR_SWITCH
	Jaguar *shooter_motor_front;
	Jaguar *shooter_motor_back;
#else
	Talon *shooter_motor_front;
	Talon *shooter_motor_back;
#endif
	Victor *climbing_motor;
#if DUMB_DRIVE_CODE
	Victor *left_drive_motor_A;
	Victor *left_drive_motor_B;
	Victor *right_drive_motor_A;
	Victor *right_drive_motor_B;
#endif

	//Limit Swithes
	DigitalInput *top_claw_limit_switch;
	DigitalInput *bottom_claw_limit_switch;

	//Encoders
	//Encoder *RPS_encoder_1;
	Encoder *front_shooter_encoder;
	Encoder *back_shooter_encoder;

	//Solenoids
	Solenoid *left_shift_solenoid_1;
	Solenoid *left_shift_solenoid_2;
	Solenoid *shooter_angle_1;
	Solenoid *shooter_angle_2;
	Solenoid *shooter_fire_piston_A;
	Solenoid *shooter_fire_piston_B;

	//Timer
	Timer *shooter_piston_timer;
	Timer *loop_time_measure_timer;
	Timer *VC_timer;
	Timer *shooter_reset;
	Timer *pid_code_timer;
	Timer *autonomous_timer;
	Timer *error_timer;
	Timer *retraction_timer;
	Timer *stabilizing_timer;
	Timer *override_timer;
	Timer *shooter_stop_timer;

	//Compressor
	Compressor *compressor1;

#if CAMERA
	//Camera
	AxisCamera *camera;
	ColorImage *image;
#endif

	//Driver Station
	DriverStationLCD *dsLCD;

	//Variables
	int test_encoder_value;
	int total_test_encoder_value;
	int second_count;
	int old_encoder_value;
	int additive_error;
	int counter;
	int number;
	int average_counter;
	int cycle_counter;

	float prev_desired_RPS;
	float RPS_back;
	float RPS_front;
	float turret_speed;
	float set_speed;
	float speed2;
	float old_RPS;
	float new_RPS;
	float average;
	float test_speed;
	float RPS_speed_front;
	float RPS_speed_back;
	float divided;
	float ROC;
	float error_front;
	float error_back;
	float integral_front;
	float integral_back;
	float derivitive_front;
	float derivitive_back;
	float prev_error_front;
	float prev_error_back;
	float shooter_motor_back_RPS;
	float shooter_motor_front_RPS;
	float accumerror;
	float VC_error;
	float preverror;
	float current;
	float dt;
	float P;
	float I;
	float D;
	float F;
	float out;
	float desired_RPS_control;

	double old_timer;

	bool first_press;
	bool arcadedrive;
	bool claw_;
	bool claw_go_down;
	bool button;
	bool pickup_on;
	bool switch_on;
	bool kicker_piston_on;
	bool kicker_button_on;
	bool constant_;
	bool constant_desired_RPS;
	bool test_RPS;
	bool slow_control;

public:
	RobotDemo()
	{
#if YEAR_2013
		drive = new RobotDrive(left_drive_motor_A_PWM, left_drive_motor_B_PWM,
				right_drive_motor_A_PWM, right_drive_motor_B_PWM);
		drive->SetExpiration(15);
		drive->SetSafetyEnabled(false); 
#endif
	
		drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		//Joystick
		//ds = new DriverStation();
		drive_stick_sec = new Joystick(right_stick);
		drive_stick_prim = new Joystick(left_stick);
		operator_stick = new Joystick(operator_joystick);
		//Motors
#if JAGUAR_SWITCH
		shooter_motor_front = new Jaguar(shooter_front_motor);
		shooter_motor_back = new Jaguar(shooter_back_motor);
#else
		shooter_motor_front = new Talon(shooter_front_motor);
		shooter_motor_back = new Talon(shooter_back_motor);
#endif
#if DUMB_DRIVE_CODE
		left_drive_motor_A = new Victor(left_drive_motor_A_PWM);
		left_drive_motor_B = new Victor(left_drive_motor_B_PWM);
		right_drive_motor_A = new Victor(right_drive_motor_A_PWM);
		right_drive_motor_B = new Victor(right_drive_motor_B_PWM);
#endif
#if YEAR_2013
		climbing_motor = new Victor(climbing_motor_PWM);
#endif
		//limit switches
		top_claw_limit_switch = new DigitalInput(top_claw_limit_switch_port);
		bottom_claw_limit_switch = new DigitalInput(
				bottom_claw_limit_switch_port);

		//Encoders
		front_shooter_encoder = new Encoder(shooter_motor_front_encoder_A_port,
				shooter_motor_front_encoder_B_port, false);
		back_shooter_encoder = new Encoder(shooter_motor_back_encoder_A_port,
				shooter_motor_back_encoder_B_port, false);

		//solenoids
		shooter_angle_1 = new Solenoid(SHOOTER_ANGLE_SOLENOID_1);
		shooter_angle_2 = new Solenoid(SHOOTER_ANGLE_SOLENOID_2);
		shooter_fire_piston_A = new Solenoid(shooter_fire_piston_solenoid_A);
		shooter_fire_piston_B = new Solenoid(shooter_fire_piston_solenoid_B);
		//Timers
		shooter_piston_timer = new Timer();
		VC_timer = new Timer();
		loop_time_measure_timer = new Timer();
		shooter_reset = new Timer();
		pid_code_timer = new Timer();
		autonomous_timer = new Timer();
		error_timer = new Timer();
		retraction_timer = new Timer();
		override_timer = new Timer();
		stabilizing_timer = new Timer();
		shooter_stop_timer = new Timer();
		//Compressor
		compressor1 = new Compressor(PRESSURE_SWITCH, compressor_enable);
#if CAMERA
		//Camera
		camera = &(AxisCamera::GetInstance(AxisCameraIpAddr));
		camera->WriteResolution(AxisCamera::kResolution_640x480);
		AxisCamera::GetInstance();
#endif

		//Function starter
		compressor1 ->Start();
		//float RPS;
		//PIDController pid1 (0.1 , 0.001 ,0.0 , &RPS , test_motor );
		loop_time_measure_timer ->Start();
		VC_timer ->Start();
		//Variable Initialization
		arcadedrive = true;
		test_encoder_value = 0;
		total_test_encoder_value = 0;
		old_RPS = 0;
		new_RPS = 0;
		second_count = 0;
		set_speed = 0.5;
		cycle_counter = 0;
		//test_motor ->Set(set_speed);
		// float RPS;
		//int old_encoder_value;
		//double old_timer;
		speed2 = 0.0;
		test_speed = 0.3;
		button = false;
		pickup_on = false;
		switch_on = false;
		kicker_piston_on = false;
		kicker_button_on = false;
		average_counter = 0;
		counter = 1;
		number = 1;
		additive_error = 0;
		prev_error_front = 0;
		prev_desired_RPS = 0;
		ROC = 0;
		claw_ = false;
		claw_go_down = false;
		constant_ = false;
		constant_desired_RPS = false;
		divided = 0;
		//RPS_encoder_1 = new Encoder(9, 10);// for 2012 robot
		shooter_motor_back_RPS = shooter_motor_back->Get();
		//RPS_encoder_1 -> SetDistancePerPulse(1 / 250);//TODO figure out how this works
		first_press = true;
		test_RPS = false;
		desired_RPS_control = 0.0;
		slow_control = 0;
		//RPS_encoder_1 ->Start();
		pid_code_timer ->Start();
		front_shooter_encoder->Start();
		back_shooter_encoder->Start();
		autonomous_timer ->Start();
		error_timer ->Start();
		retraction_timer->Start();
		stabilizing_timer->Start();
		override_timer->Start();
	}

	void DriverLCD();
	float lookup_distance_array(float distance);
	void interpolated_test_code();
	void pneumatic_feeder_code();
	void pneumatic_shooter_angler_code();
	void RPS_control_code(float desired_RPS);
	void arcade_tank_code();
	float cin_code_get();
	void z_axis_control();
	void climber_code();
	void camera_test();
	void printfs();
	void dumb_drive_code();
	void integral_reset();
	void dump_code();
	void constant_RPS_code();
	void dumb_climber_code();
	void intelligent_shooter();
	void climber_state();
	void pointer_test(Talon *Test_motor);
	/*
	 * This function is called once each time the robot enters autonomous mode.
	 */

	void Autonomous()
	{
		GetWatchdog().SetEnabled(true);
		autonomous_timer->Reset();
#if TIMER_RESET
		pid_code_timer->Reset();
#endif
		front_shooter_encoder->Reset();
		back_shooter_encoder->Reset();
		shooter_angle_1 ->Set(true);
		shooter_angle_2 ->Set(false);
		retraction_timer->Reset();
		stabilizing_timer->Reset();
		override_timer->Reset();
		//shooter_motor_front->Set(0.8);
		//shooter_motor_front->Set(0.8);
		integral_back = 0.0;
		integral_front = 0.0;
		error_back = 0.0;
		error_front = 0.0;
		desired_RPS_control = back_position_RPS_1;
		smart_autonomous_state = unstable;//default is unstable


		while (IsAutonomous() && IsEnabled())
		{
			printfs();
			GetWatchdog().Feed();
			RPS_control_code(desired_RPS_control);
			Wait(.001);
			if (autonomous_timer->Get() >= first_fire && autonomous_timer->Get() <(first_fire + 1))//fire
			{
				shooter_fire_piston_A ->Set(false);
				shooter_fire_piston_B ->Set(true);
			}
			if (autonomous_timer->Get() >= first_retract && autonomous_timer->Get() < (first_retract + 2))//retract

			{
				shooter_fire_piston_A ->Set(true);
				shooter_fire_piston_B ->Set(false);
			}
			if (autonomous_timer->Get() >= second_fire && autonomous_timer->Get() < (second_fire + 1))//fire

			{
				shooter_fire_piston_A ->Set(false);
				shooter_fire_piston_B ->Set(true);
			}
			if (autonomous_timer->Get() >= second_retract && autonomous_timer->Get() < (second_retract +2))//retract

			{
				shooter_fire_piston_A ->Set(true);
				shooter_fire_piston_B ->Set(false);
			}
			if (autonomous_timer->Get() >= third_fire && autonomous_timer->Get() < (third_fire + 1))//fire

			{
				shooter_fire_piston_A ->Set(false);
				shooter_fire_piston_B ->Set(true);
			}
			if (autonomous_timer->Get() >= third_retract)//final retract

			{
				shooter_fire_piston_A ->Set(true);
				shooter_fire_piston_B ->Set(false);
			}
		}

		autonomous_timer->Reset();
	}
	void OperatorControl()//TODO remember that this is the beginning of operator controll
	{
		printf(
				"****************************VERSION .00017****************************\n");
#if CAMERA
		AxisCamera &camera = AxisCamera::GetInstance("10.26.43.11");
#endif
		GetWatchdog().SetEnabled(true);
		shooter_reset ->Start();
#if TIMER_RESET
		pid_code_timer->Reset();
#endif
		front_shooter_encoder->Reset();
		back_shooter_encoder->Reset();
		/*float prev_error_front = 0;
		 float prev_error_back = 0;
		 int test_back = 5;
		 int test_front = 6;*/
		integral_back = 0.0;
		integral_front = 0.0;
		desired_RPS_control = 0.0;
		//RETRACTS PISTON WHEN TELEOP STARTS

		override_timer->Reset();
		stabilizing_timer ->Reset();
		retraction_timer ->Reset();

		shooter_fire_piston_A ->Set(true);
		shooter_fire_piston_B ->Set(false);
		shooter_stop_timer->Start();
		while (IsOperatorControl() && IsEnabled())
		{
			GetWatchdog().Feed();
			//---------------------Display Output---------------------------
			dsLCD = DriverStationLCD::GetInstance();
			DriverLCD();
			//printf("PRINTFS\n");
			printfs();

			//-------------------------Climber-----------------------------
			//climber_code();
			//dumb_climber_code();
			climber_state();

			//---------------------------Drive-----------------------------
			arcade_tank_code();
			constant_RPS_code();
			//dumb_drive_code();   // In case our smart code doesnt work

			//--------------------------PID-------------------------------
			integral_reset();

			//-------------------------Shooter----------------------------
			pneumatic_shooter_angler_code();
			pneumatic_feeder_code();
			//dump_code();
			//intelligent_shooter();

			//-------------------------Test Code--------------------------
			//camera_test();
			//cin_code_get();
			//prev_error_back = RPS_control_code(shooter_motor_back, back_shooter_encoder, prev_error_back, desired_RPS_control);
			//interpolated_test_code();
			//pointer_test(shooter_motor_front);
			//PIDController(first_pterm, iterm, dterm, front_shooter_encoder, shooter_motor_front);

			//------------cRIO Housekeeping Timing-MUST HAVE--------------
			Wait(0.005);

		}//while operator
	}//operator ctrl
};//class robot demo
void RobotDemo::DriverLCD()
{
	if (cycle_counter >= 50)
	{
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "RPS Back:%f  ",
				RPS_back);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "RPS Front:%f  ",
				RPS_front);
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "RPS DRPS:%f  ",
				desired_RPS_control);
#if 0
		if (shooter_fire_piston_A->Get())
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,"Fire   ");
		}
		else
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,"Retracting...   ");
		}
#endif
		//dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,"TopLS:%i   BotLS:%i   ", top_claw_limit_switch->Get(),
		//	bottom_claw_limit_switch ->Get());
		if (top_claw_limit_switch->Get())
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "!TOP");
		}
		else if (!bottom_claw_limit_switch->Get())
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "!BOTTOM");
		}
		else
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Neither");
		}
		if (shooter_angle_1->Get())
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Up   ");
		}
		else
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Down   ");
		}
		if (arcadedrive)
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Arcade   ");
		}
		else
		{
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Tank   ");
		}
		dsLCD->UpdateLCD();
		//cycle_counter = 0;
	}
	//cycle_counter++;
}
void RobotDemo::climber_state()
{
	switch (smart_climber_state)
	{
	case sendup:
		if (top_claw_limit_switch->Get() != 1)
		{
			climbing_motor ->Set(1.0);
		}
		else
		{
			smart_climber_state = stop;
		}
		if (drive_stick_prim ->GetRawButton(3))
		{
			smart_climber_state = go_up;
		}
		if (drive_stick_prim ->GetRawButton(2))
		{
			smart_climber_state = go_down;
		}
		break;

	case go_up:
		if (drive_stick_prim ->GetRawButton(3)
				&& bottom_claw_limit_switch->Get() != 1)
		{
			climbing_motor ->Set(1.0);
		}
		else
		{
			smart_climber_state = stop;
		}
		if (drive_stick_prim ->GetRawButton(2)
				&& bottom_claw_limit_switch->Get() != 1)
		{
			smart_climber_state = go_down;
		}
		break;

	case go_down:
		if (drive_stick_prim ->GetRawButton(2)
				&& bottom_claw_limit_switch->Get() != 1)
		{
			climbing_motor ->Set(-1.0);
		}
		else
		{
			smart_climber_state = stop;
		}
		break;

	case stop:
		climbing_motor ->Set(0.0);
		if (drive_stick_prim ->GetRawButton(2)
				&& bottom_claw_limit_switch->Get() != 1)
		{
			smart_climber_state = go_down;
		}
		if (drive_stick_prim ->GetRawButton(3) && top_claw_limit_switch->Get()
				!= 1)
		{
			smart_climber_state = go_up;
		}
		if (drive_stick_prim ->GetRawButton(10) && top_claw_limit_switch->Get()
				!= 1)
		{
			smart_climber_state = sendup;
		}

	}

}

void RobotDemo::intelligent_shooter()
{
	RPS_control_code(37.5);

	switch (smart_autonomous_state)
	{
	case unstable:
		cout << "State Unstable" << endl;
		printf("%i %i", fabs(error_back) < 4, fabs(error_front) < 4);
		if (fabs(error_back) < 4 || fabs(error_front) < 4)
		{
			smart_autonomous_state = stabilizing;
			stabilizing_timer->Reset();
		}
		if (override_timer->Get() > 4)
		{
			smart_autonomous_state = fire;
		}

		break;
	case stabilizing:
		cout << "State Stabilizing   " << endl;
		if (stabilizing_timer->Get() > 1 || override_timer->Get() > 4)
		{
			smart_autonomous_state = fire;
		}
		if (fabs(error_back) > 4 || fabs(error_front) > 4)
		{
			smart_autonomous_state = unstable;
		}
		break;

	case fire:
		cout << "State Firing   " << endl;
		shooter_fire_piston_A ->Set(false);//piston -->
		shooter_fire_piston_B ->Set(true);
		smart_autonomous_state = retracting;
		retraction_timer->Reset();
		break;
	case retracting:
		cout << "State Retracting   " << endl;
		if (retraction_timer->Get() > 1)
		{
			smart_autonomous_state = retract;
			break;
		}
		break;
	case retract:
		cout << "State Has Retracted   " << endl;
		shooter_fire_piston_A ->Set(true);//piston <--
		shooter_fire_piston_B ->Set(false);
		smart_autonomous_state = unstable;
		override_timer->Reset();
		break;
	}
	printf("%i\n", smart_autonomous_state);
}

void RobotDemo::constant_RPS_code()
{
	if (operator_stick->GetRawButton(front_position_button))
	{
		desired_RPS_control = front_position_RPS;
	}
	else
	{
		if (operator_stick->GetRawButton(back_position_button_1))
		{
			desired_RPS_control = back_position_RPS_1;
		}
		else if (operator_stick->GetRawButton(back_position_button_2))
		{
			desired_RPS_control = back_position_RPS_2;
		}
		else if (operator_stick->GetRawButton(back_position_button_3))
		{
			desired_RPS_control = back_position_RPS_3;
		}
		else if (operator_stick->GetRawButton(back_position_button_4))
		{
			desired_RPS_control = back_position_RPS_4;
		}
		else if (operator_stick->GetRawButton(back_position_button_5))
		{
			desired_RPS_control = back_position_RPS_5;
		}
		else
		{
			if (operator_stick->GetRawButton(dumper_button_A)
					|| operator_stick->GetRawButton(dumper_button_B))
			{
				desired_RPS_control = dumper_RPS;
			}
			else
			{
				desired_RPS_control = ((-operator_stick->GetZ() + 1) / 2) * 75;
			}
		}
	}

	
		RPS_control_code(desired_RPS_control);

}

float RobotDemo::lookup_distance_array(float distance)// this function gets
{
	if (distance > 31)
		return distance_lookup_array[31];
	if (distance < 0)
		return distance_lookup_array[0];
	int lower_distance = int(distance);
	int upper_distance = lower_distance + 1;
	float lower_value = distance_lookup_array[lower_distance];
	float upper_value = distance_lookup_array[upper_distance];
	float difference = distance - lower_value;
	float interpolation = lower_value + difference
			* (upper_value - lower_value);
	return interpolation;
}
void RobotDemo::interpolated_test_code()
{
	float Test_Distance_Input = (((drive_stick_sec->GetZ() + 1) / 2.0) * 33.0)
			- 1;
	float Test_Distance_Output = lookup_distance_array(Test_Distance_Input);
	printf("input:%f output:%f\n", Test_Distance_Input, Test_Distance_Output);
}

void RobotDemo::dumb_climber_code()
{
	//printf("Top:%i   Bottom:%i  " , top_claw_limit_switch->Get() , bottom_claw_limit_switch->Get());
	if (drive_stick_prim ->GetRawButton(climber_hold_up))
	{
		if (top_claw_limit_switch->Get() == 1)
		{
			climbing_motor->Set(0.0);
			//printf("STOPPED\n");
		}
		else
		{
			climbing_motor->Set(1);
			//printf("GOING UP\n");
		}
	} // not climber hold up

	else if (drive_stick_prim->GetRawButton(climber_hold_down))
	{
		if (bottom_claw_limit_switch->Get() == 0)
		{
			climbing_motor->Set(0.0);
			//printf("STOPED\n");
		}
		else
		{
			climbing_motor->Set(-1);
			//printf("GOING DOWN\n");
		}
	} // hold down button
	else
	{ // no js buttons pushed
		climbing_motor->Set(0.0);
	}
}
void RobotDemo::pneumatic_feeder_code()
{
	if (operator_stick ->GetRawButton(shooter_piston_button)) //Button 1                                
	{
		if (kicker_button_on == false)
		{
			shooter_reset ->Reset();
			kicker_button_on = true;
			//kicker_piston_on = false;
			shooter_piston_timer->Start();

			shooter_fire_piston_A ->Set(false);//pushes
			shooter_fire_piston_B ->Set(true);

			cout << "out" << endl;
		}
	}
	else
	{
		kicker_button_on = false;
	}
	if (shooter_piston_timer->Get() >= shooter_piston_delay)
	{
		shooter_fire_piston_A ->Set(true);//retracts
		shooter_fire_piston_B ->Set(false);

		shooter_piston_timer ->Reset();
		shooter_piston_timer ->Stop();

		cout << "back" << endl;
	}
}

void RobotDemo::pneumatic_shooter_angler_code()
{
	if (operator_stick ->GetRawButton(tilt_up_button))
	{
		shooter_angle_1 ->Set(true);
		shooter_angle_2 ->Set(false);
	}
	else
	{
		if (operator_stick ->GetRawButton(tilt_down_button))
		{
			shooter_angle_1 ->Set(false);
			shooter_angle_2 ->Set(true);
		}

	}
}
// the following is the motor set code


void RobotDemo::RPS_control_code(float desired_RPS)
// THE 360 COUNT PER REVOLUTION ENCODERS ARE GOOD FOR 10K RPM, BUT THE CIM motor MAXIMUM SPEED IS APPROXIMATELY 5K.
{
#if 1
	/* If the difference between the old and new RPS (typ ~35) by greater than 1 either way,
	 * reset the ingegral terms to prevent stale values from affecting our PID calculations
	 */
	
	if (desired_RPS != prev_desired_RPS)
	{
		if (fabs(prev_desired_RPS - desired_RPS) >= 1.0)
		{
			integral_back = 0;
			integral_front = 0;
		}
		cout << desired_RPS << " " << prev_desired_RPS << endl;
		prev_desired_RPS = desired_RPS;
	}
#endif
	//motor = new Victor(motor_pwm);
	//test_encoder = new Encoder(encoder_pwm1, encoder_pwm2, true);
	if ((pid_code_timer->Get()) >= pidtime)
	{
		float actual_time = pid_code_timer ->Get();
		prev_error_back = error_back;
		prev_error_front = error_front;
		RPS_back = (back_shooter_encoder->Get() / 360.0) / actual_time;
		RPS_front = (front_shooter_encoder->Get() / 360.0) / actual_time;
		error_back = desired_RPS - RPS_back;
		error_front = desired_RPS - RPS_front;
		integral_back = integral_back + (error_back * actual_time);
		integral_front = integral_front + (error_front * actual_time);
		derivitive_back = (error_back - prev_error_back) / actual_time;
		derivitive_front = (error_front - prev_error_front) / actual_time;

		//take error and set it to motors
		{
			RPS_speed_back = (error_back * first_pterm) + (integral_back
					* iterm) + (derivitive_back * dterm);
			RPS_speed_front = (error_front * first_pterm) + (integral_front
					* iterm) + (derivitive_front * dterm);
		}

		if (RPS_speed_back > 1)
		{
			RPS_speed_back = 1;
		}
		if (RPS_speed_front > 1)
		{
			RPS_speed_front = 1;
		}
		if (RPS_speed_back < 0)
		{
			RPS_speed_back = 0;
		}
		if (RPS_speed_front < 0)
		{
			RPS_speed_front = 0;
		}
		//cout << RPS_speed << endl;
		//cout << autonomous_timer->Get() << "   RPS_back: " << RPS_back

		//<< "   RPS_front: " << RPS_front << "   Desired RPS:"
		//<< autonomous_desired_RPS << endl;
		shooter_motor_back ->Set(RPS_speed_back);
		shooter_motor_front ->Set(RPS_speed_front);
		counter++;
#if 0
		if (counter == 5)
		{
			counter = 1;
			printf(
					"                                         %i)%f   %f   %f   %f\n",
					number, RPS, RPS_speed, error,
					shooter_motor_back->Get());
			number++;
		}
		else

		{
			printf("%f   %f   %f   %f\n", RPS, RPS_speed, error,
					shooter_motor_back->Get());
		}
#endif
		pid_code_timer->Reset();
		back_shooter_encoder ->Reset();
		front_shooter_encoder ->Reset();
	}
}
void RobotDemo::pointer_test(Talon *Test_motor)
{
	Test_motor->Set(0.0);
	return;
}
void RobotDemo::arcade_tank_code()
{
	if (drive_stick_prim ->GetRawButton(arcade_button))
	{
		arcadedrive = true;
	}
	else
	{
		if (drive_stick_prim ->GetRawButton(tank_button))
		{
			arcadedrive = false;
		}
	}
	if (drive_stick_prim->GetRawButton(2) || drive_stick_sec->GetRawButton(2))
	{
		slow_control = true;
	}
	else
	{
		slow_control = false;
	}
	if (arcadedrive)
	{
		if (slow_control)
		{
			drive->ArcadeDrive(Adjustment_Speed * (drive_stick_prim->GetY()),
					Adjustment_Speed * (drive_stick_prim->GetX()), true);
		}
		else
		{
			drive->ArcadeDrive((drive_stick_prim->GetY()),
					(drive_stick_prim->GetX()), true);
		}
	}
	else
	{
		if (slow_control)
		{
			drive->TankDrive(Adjustment_Speed * (drive_stick_prim->GetY()),
					Adjustment_Speed * (drive_stick_sec->GetY()), true);
		}
		else
		{
			drive->TankDrive((drive_stick_prim->GetY()),
					(drive_stick_sec->GetY()), true);
		}
	}
}
float RobotDemo::cin_code_get()
{
	float cin_test_var;
	printf("cin\n");
	cin >> cin_test_var;
	cout << cin_test_var << endl;
	return cin_test_var;

}
void RobotDemo::z_axis_control()
{
	//shooter_motor_front->Set((-operator_stick->GetZ() + 1.0) / 2.0);//Shooter motors must always be set to positive values
	//shooter_motor_back->Set((-operator_stick->GetZ() + 1.0) / 2.0);
	if (!constant_desired_RPS)
	{
		desired_RPS_control = ((-operator_stick->GetZ() + 1) / 2) * 75;
		RPS_control_code(desired_RPS_control);
	}

}
void RobotDemo::climber_code()
{
	if (drive_stick_prim ->GetRawButton(climber_off_button))
	{
		climbing_motor-> Set(0.0);
		claw_go_down = false;
		claw_ = false;

	}
	else
	{
		if (drive_stick_prim ->GetRawButton(climber_hold_up))
		{
			claw_go_down = false;
			claw_ = false;
			if (top_claw_limit_switch->Get())
			{
				climbing_motor->Set(0.0);
			}
			else
			{
				climbing_motor->Set(1);
			}

		}// hold up
		else
		{
			if (drive_stick_prim ->GetRawButton(climber_hold_down))
			{
				claw_go_down = false;
				claw_ = false;
				if (bottom_claw_limit_switch->Get() == false)
				{
					climbing_motor ->Set(0.0);
				}
				else
				{
					climbing_motor ->Set(-1);
				}
			} // hold down
			else if (drive_stick_sec ->GetRawButton(climber_send_top))
			{
				claw_ = true;
				claw_go_down = false;
			}
			else if (drive_stick_sec ->GetRawButton(climber_send_bottom))
			{
				claw_go_down = true;
				claw_ = false;
			}
			else
			{ // no holding or sending
				if ((claw_ == false) && (claw_go_down == false))
				{
					climbing_motor ->Set(0.0);
				}
			}

		} // no climber hold up button

	} // no climber off button

	if (claw_go_down)
	{
		if (bottom_claw_limit_switch->Get() == 1)
		{
			climbing_motor ->Set(-1);
		}
		else
		{
			climbing_motor ->Set(0.0);
		}

	}
	if (claw_)
	{
		if (top_claw_limit_switch->Get() == 0)
		{
			climbing_motor->Set(1);
		}
		else
		{
			climbing_motor ->Set(0.0);
		}
	}
}
void RobotDemo::camera_test()
{
#if CAMERA
	image = camera->GetImage();

	cout << image << endl;
	delete image;
#endif
}
void RobotDemo::dump_code()
{
	if (operator_stick ->GetRawButton(8))
	{
		shooter_motor_front ->Set(0.3);
		shooter_motor_back ->Set(0.3);
	}
}
void RobotDemo::printfs()
{
	//printf("OUT LOOP\n");
	if (cycle_counter >= 70)
	{
		printf("RPS Back:%f   RPS Front:%f   %f    ", RPS_back, RPS_front,
				desired_RPS_control);
		printf("LS:%i %i   ", top_claw_limit_switch->Get(),
				bottom_claw_limit_switch->Get());
		//printf("Climb:%f   " , climbing_motor->Get());
		//printf("Shooters:%f   %f   ", shooter_motor_front ->Get(),
		//shooter_motor_back ->Get());

		if (shooter_angle_1->Get())
		{
			printf("Up   ");
		}
		else
		{
			printf("Down   ");
		}
		if (arcadedrive)
		{
			printf("Arcade   ");
		}
		else
		{
			printf("Tank   ");
		}
		printf("\n");

		cycle_counter = 0;
	}
	cycle_counter++;

}
void RobotDemo::dumb_drive_code()
{
#if DUMB_DRIVE_CODE
	left_drive_motor_A->Set(-drive_stick_sec ->GetY());
	left_drive_motor_B->Set(-drive_stick_sec->GetY());
	right_drive_motor_A->Set(drive_stick_prim->GetY());
	right_drive_motor_B->Set(drive_stick_prim->GetY());
#endif
}

void RobotDemo::integral_reset()
{
	if (operator_stick ->GetRawButton(10))
	{
		integral_back = 0;
		integral_front = 0;
	}
}
START_ROBOT_CLASS(RobotDemo)
;
