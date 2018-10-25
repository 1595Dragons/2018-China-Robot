#include "WPILib.h"
#include "ctre/Phoenix.h"
// For some reason we dont have the navX on this robot...
// #include "AHRS.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

class Robot: public IterativeRobot {

private:
	int kTimeoutMs = 10;

	// Get the CAN bus addresses for the motors
	// 7 is either wrist or one of the intakes, same with 11 and 8
	int lift1Address = 5,
			lift2Address = 6,
			lift3Address = 4,
			wristAddress = 11,
			lDrive1Address = 3, // Has encoder
			lDrive2Address = 2,
			rDrive1Address = 10, // Has encoder
			rDrive2Address = 9,
			lIntakeAddress = 8,
			rIntakeAddress = 7,
			DriverJoystickAddress = 0,
			OperatorJoystickAddress = 1;

	// TODO: When turning some of the drive motors move incorrectly...

	TalonSRX * lift1 = new TalonSRX(lift1Address);
	TalonSRX * lift2 = new TalonSRX(lift2Address);
	TalonSRX * lift3 = new TalonSRX(lift3Address);
	TalonSRX * wrist = new TalonSRX(wristAddress);

	TalonSRX * lDrive1 = new TalonSRX(lDrive1Address);
	TalonSRX * rDrive1 = new TalonSRX(rDrive1Address);
	TalonSRX * lDrive2 = new TalonSRX(lDrive2Address);
	TalonSRX * rDrive2 = new TalonSRX(rDrive2Address);
	TalonSRX * lIntake = new TalonSRX(lIntakeAddress);
	TalonSRX * rIntake = new TalonSRX(rIntakeAddress);

	// Setup joysticks
	Joystick * dr = new Joystick(DriverJoystickAddress);
	Joystick * op = new Joystick(OperatorJoystickAddress);

	//DoubleSolenoid * PTO = new DoubleSolenoid(0,1);
	double lIntakePow = 0;
	double rIntakePow = 0;
	double wristPow = 0;
	double lastFrameWristPow = 0;
	double lastFrameTime = 0;
	double liftPow = 0;
	double drLY = 0;
	double drRX = 0;
	double opLY = 0;
	double opRY = 0;
	double angSetPoint = 0;
	double distSetPoint = 0;

	double iTerm = 0;
	double lastAng = 0;
	double lastTime = 0;

	Timer stageTimer;

	int encToDist = 100;
	int autoStage = 0;
	bool turning = false;
	bool driving = false;
	bool outTake = false;
	bool liftSensor = false;
	double liftSetPoint = 0;
	double wristSetPoint = .25 * 5 * 10;
	Timer teleOpTimer;
	Timer autoTimer;
	Preferences * prefs;
	std::string gameData;
	SendableChooser<std::string> chooser;
	SendableChooser<std::string> side;
	SendableChooser<std::string> practice;

	int encToInch = 100;
	double joyAng = 0;
	double turnError = 0;

	//AHRS * ahrs;

	double PID(double setPoint, double current, double last, double kP,
			double kI, double kD, double iZone, double timeSinceLast) {
		double error = setPoint - current;
		double kTerm = kI * setPoint / current;
		if (abs(error) < iZone) {
			iTerm += kI * current * timeSinceLast;
		} else {
			iTerm = 0;
		}
		double dTerm = kD * ((current - last) / timeSinceLast);
		return kTerm + iTerm + dTerm;

	}

	void RobotInit() {
		prefs = Preferences::GetInstance();
		chooser.AddDefault("driveForward", "driveForward");
		chooser.AddObject("switchAdaptive", "switchAdaptive");
		chooser.AddObject("switchMid", "switchMid");
		chooser.AddObject("nothing", "nothing");

		side.AddDefault("L", "L");
		side.AddObject("R", "R");

		practice.AddDefault("No", "No");
		practice.AddObject("Yes", "Yes");

		SmartDashboard::PutData("Auto Modes", &chooser);
		SmartDashboard::PutData("Side", &side);
		SmartDashboard::PutData("Practice", &practice);

		int kTimeoutMs = 10;
		//Drive settings
		rDrive2->Set(ControlMode::Follower, 3);
		lDrive2->Set(ControlMode::Follower, 10);
		lDrive1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lDrive2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		rDrive1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		rDrive2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lift1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lift2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lift3->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lift2->Set(ControlMode::Follower, 4);
		lift3->Set(ControlMode::Follower, 4);
		rDrive1->SetInverted(true);
		rDrive2->SetInverted(true);
		lDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				kTimeoutMs);
		rDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				kTimeoutMs);
		lDrive1->SetSensorPhase(true);
		rDrive1->SetSensorPhase(true);
		//wrist->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,kTimeoutMs);
		if (liftSensor) {
			lift1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
					kTimeoutMs);
		}

		//rIntake->Set(ControlMode::Follower, 8); //follow lIntake
		//rIntake->SetInverted(true);
		wrist->SetInverted(true);
		/*
		 try {
		 ahrs = new AHRS(I2C::Port::kMXP);
		 } catch (std::exception& ex) {
		 std::string err_string = "Error instantiating navX MXP:  ";
		 err_string += ex.what();
		 DriverStation::ReportError(err_string.c_str());
		 }
		 */
	}

	void AutonomousInit() {
		//ahrs->ZeroYaw();
		autoStage = 0;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		autoTimer.Start();
		stageTimer.Start();
		lDrive1->Config_kP(0, .4, kTimeoutMs);
		lDrive1->Config_kI(0, .005, kTimeoutMs);
		lDrive1->Config_kD(0, 13, kTimeoutMs);
		rDrive1->Config_kP(0, .4, kTimeoutMs);
		rDrive1->Config_kI(0, .005, kTimeoutMs);
		rDrive1->Config_kD(0, 13, kTimeoutMs);
		if (liftSensor) {
			lift1->SetSelectedSensorPosition(0, 0, kTimeoutMs);
		}
		//wrist->SetSelectedSensorPosition(0, 0, kTimeoutMs);
		lDrive1->SetSelectedSensorPosition(0, 0, kTimeoutMs);
		rDrive1->SetSelectedSensorPosition(0, 0, kTimeoutMs);
		if (liftSensor) {
			lift1->Config_kP(0, 1, kTimeoutMs);
			lift1->Config_kI(0, .1, kTimeoutMs);
			lift1->Config_kD(0, 1, kTimeoutMs);
		}
		/*wrist->Config_kP(0, 1, kTimeoutMs);
		 wrist->Config_kI(0, .1, kTimeoutMs);
		 wrist->Config_kD(0, 1, kTimeoutMs);*/
	}

	void AutonomousPeriodic() {
		if (chooser.GetSelected() != "nothing") {
			if (chooser.GetSelected() == "driveForward") {
				if (autoStage == 0) {
					driving = false;
					turning = false;
					//distSetPoint = 100;
					lDrive1->Set(ControlMode::Position, -1 * 100 * encToDist);// - (ahrs->GetYaw() / 90));
					rDrive1->Set(ControlMode::Position, -1 * 100 * encToDist);// + (ahrs->GetYaw() / 90));
				}
			} else if (chooser.GetSelected() == "SwitchAdaptive") {
				if (autoStage == 0) {
					// = 50;
					lDrive1->Set(ControlMode::Position, -1 * 50 * encToDist);// - (ahrs->GetYaw() / 90));
					rDrive1->Set(ControlMode::Position, -1 * 50 * encToDist);// + (ahrs->GetYaw() / 90));
					driving = false;
					turning = false;
				}
				if (autoStage == 1 & gameData[0] == 'R'
						& side.GetSelected() == "R") {
					driving = false;
					//wrist->Set(ControlMode::PercentOutput, .8);
					if (stageTimer.Get() < .3) {
						wristPow = -.8;
					} else {
						wristPow = 0;
						outTake = true;
					}
				}
				if (autoStage == 1 & gameData[0] == 'L'
						& side.GetSelected() == "L") {
					driving = false;
					//wrist->Set(ControlMode::PercentOutput, .8);
					if (stageTimer.Get() < .3) {
						wristPow = -.8;
					} else {
						wristPow = 0;
						outTake = true;
					}
				}
				if (/*practice.GetSelected() != "No" & */autoStage == 2) {
					driving = false;
					//wristPow = -.8;//->Set(ControlMode::PercentOutput, -.8);
					if (stageTimer.Get() < 2) {
						wristPow = -.8;
					} else {
						wristPow = 0;
						outTake = true;
					}
				}
			}
			//wrist->Set(ControlMode::PercentOutput, .8);
			/*else if(chooser.GetSelected() == "switchMid" & liftSensor){
			 if(autoStage == 0){
			 driving = true;
			 turning = false;
			 distSetPoint = 70;
			 }
			 if(autoStage == 1){
			 driving = false;
			 turning = true;
			 //liftSetPoint = 50;
			 //wristSetPoint = 2;
			 if(gameData[0] == 'R'){
			 angSetPoint = 90;
			 }
			 else{
			 angSetPoint = -90;
			 }
			 }
			 if(autoStage == 2){
			 driving = true;
			 turning = false;
			 distSetPoint = 60;
			 }
			 if(autoStage == 3){
			 driving = false;
			 turning = true;
			 if(gameData[0] == 'R'){
			 angSetPoint = -90;
			 }
			 else{
			 angSetPoint = 90;
			 }
			 }
			 if(autoStage == 4){
			 driving = true;
			 turning = false;
			 distSetPoint = 60;
			 }
			 if(autoStage == 5){
			 outTake = true;
			 }
			 }*/
			/*if(driving == true){
			 lDrive1->Set(ControlMode::Position, -1 * distSetPoint * encToDist);// - (ahrs->GetYaw() / 90));
			 rDrive1->Set(ControlMode::Position, -1 * distSetPoint * encToDist);// + (ahrs->GetYaw() / 90));
			 }
			 else{
			 lDrive1->Set(ControlMode::PercentOutput, PID(angSetPoint, ahrs->GetYaw(), lastAng, .6, .005, .6, 3, autoTimer.Get() - lastTime));
			 rDrive1->Set(ControlMode::PercentOutput, -1 * PID(angSetPoint, ahrs->GetYaw(), lastAng, .6, .005, .6, 3, autoTimer.Get() - lastTime));
			 }*/
			if (stageTimer.Get() > 5) {
				/*lDrive1->SetSelectedSensorPosition(0, 0, kTimeoutMs);
				 rDrive1->SetSelectedSensorPosition(0, 0, kTimeoutMs);*/
				/*distSetPoint = 0;
				 angSetPoint = 0;*/
				stageTimer.Reset();
				autoStage++;
			}
			if (outTake) {
				rIntake->Set(ControlMode::PercentOutput, 100);
				lIntake->Set(ControlMode::PercentOutput, 100);
			}
			if (liftSensor) {
				lift1->Set(ControlMode::Position, liftSetPoint);
			}
			wrist->Set(ControlMode::PercentOutput, wristPow);
			//wrist->Set(ControlMode::Position, wristSetPoint);
			//lastAng = ahrs->GetYaw();
			lastTime = autoTimer.Get();
			SmartDashboard::PutNumber("Autostage", autoStage);
			SmartDashboard::PutNumber("distSetPoint", distSetPoint);
			SmartDashboard::PutNumber("AngSetPoint", angSetPoint);
			SmartDashboard::PutNumber("AutoStage", autoStage);
			SmartDashboard::PutNumber("LiftPos",
					lift1->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("StageTimer", stageTimer.Get());
		}
	}

	void TeleopInit() {
		//ahrs->ZeroYaw();
		teleOpTimer.Start();
	}

	void TeleopPeriodic() {
		drLY = dr->GetRawAxis(1);

		drRX = dr->GetRawAxis(4);

		opLY = op->GetRawAxis(1);

		opRY = op->GetRawAxis(5);

		lDrive1->Set(ControlMode::PercentOutput, drLY + drRX * .6);
		rDrive1->Set(ControlMode::PercentOutput, drLY - drRX * .6);

		if (op->GetRawButton(1) == false) {
			lIntakePow = op->GetRawAxis(3) - op->GetRawAxis(2);
			rIntakePow = lIntakePow;
			wristPow = opLY * -1;
			liftPow = opRY;
		} else {
			lIntakePow = (op->GetRawAxis(3) - op->GetRawAxis(2)) * .5;
			rIntakePow = lIntakePow;

			liftPow = opRY * .5;
			wristPow = opLY * .5 * -1;
		}
		if (op->GetRawButton(5)) {
			lIntakePow = 1;
		}
		if (op->GetRawButton(6)) {
			rIntakePow = 1;
		}

		/*if((lastFrameWristPow - wristPow) / (lastFrameTime - teleOpTimer.Get()) > 2){//To Limit the Acceleration of the Wrist.  To adjust : 1 / Time to get to full Power (sec)
		 wristPow = lastFrameWristPow + .02;
		 }
		 if((lastFrameWristPow - wristPow) / (lastFrameTime - teleOpTimer.Get()) < -2){
		 wristPow = lastFrameWristPow - .02;
		 }*/
		if (liftSensor) {
			if (!op->GetRawButton(2) && lift1->GetSelectedSensorPosition(0) < 0
					&& opRY < 0) {
				liftPow = 0;
			}
		}
		/*if(wrist->GetSelectedSensorPosition(0) < 0 && opLY < 0){
		 wristPow = 0;
		 }*/
		if (op->GetRawButton(3)) {
			//PTO->Set(DoubleSolenoid::kForward);
		} else if (op->GetRawButton(4)) {
			//PTO->Set(DoubleSolenoid::kReverse);
		}
		lIntake->Set(ControlMode::PercentOutput, lIntakePow);
		rIntake->Set(ControlMode::PercentOutput, rIntakePow);
		wrist->Set(ControlMode::PercentOutput, wristPow);
		lift1->Set(ControlMode::PercentOutput, liftPow);

		lastFrameTime = teleOpTimer.Get();
		lastFrameWristPow = wristPow;

		//SmartDashboard::PutNumber("WristPos", wrist->GetSelectedSensorPosition(0));
		if (liftSensor) {
			SmartDashboard::PutNumber("LiftPos",
					lift1->GetSelectedSensorPosition(0));
		}
	}
};

START_ROBOT_CLASS(Robot)
