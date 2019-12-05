/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveController;

/**
 * Add your docs here.
 */
public class DriveTrainSubsytem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //toggle var to reverse direction of driving
  public boolean toggle = false;
  //ultra-sonic sensor
  //public static final AnalogInput mb1013 = new AnalogInput(RobotMap.mb1013Port);
  //angle to go to 
  double setDirection = 0;
  //error for P controller
  double error;
  //output of P contoller
  double rotOutput;
  //holds value of last heading angle
  double lastHeading;
  //string var for mode
  String mode = "";
  //for auton
  static double setAngle;

  String autoDriveState;

  // assigning motors to speed controls relative chanel
  public  static SpeedController fLeftCim = new Talon(RobotMap.fLeftCim);
  public static SpeedController bLeftCim = new Talon(RobotMap.bLeftCim);
  public static SpeedController fRightCim = new Talon(RobotMap.fRightCim);
  public static SpeedController bRightCim = new Talon(RobotMap.bRightCim);

  // creating mecanum drive object
  public static MecanumDrive mecanum_drive = new MecanumDrive(fLeftCim, bLeftCim, fRightCim, bRightCim);

  // runs method on startup? (constructer method)
  public DriveTrainSubsytem() {
    // may need to disable during P controller and for auton
    //drive.setSafetyEnabled(true);
    System.out.println("DriveSystem Started");
  }

  // robot-centric mecanum drive
  public void mecanumDriveMethod(double ySpeed, double xSpeed, double zRotation) {
    SmartDashboard.putNumber("Gyro", Robot.gyro.getAngle());
    SmartDashboard.putNumber("Gyro Ranged", getAngle());
    SmartDashboard.putNumber("Z Rotation", zRotation);
    if (Math.abs(zRotation) < (RobotMap.threshold)) {
      zRotation = 0;
    }
    if (Math.abs(ySpeed) < (RobotMap.threshold)) {
      ySpeed = 0;
    }
    if (Math.abs(xSpeed) < (RobotMap.threshold)) {
      xSpeed = 0;
    }

    mecanum_drive.driveCartesian(ySpeed * RobotMap.throttleCut, xSpeed * RobotMap.throttleCut,
        zRotation * RobotMap.throttleCut);
  }

  // field-centric "field-oriented" mecanum drive
  public void mecanumDriveGyro(final double ySpeed, final double xSpeed, final double zRotation, final double gyroAngle) {
    mecanum_drive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  public void mecanumAngleDrive(final double ySpeed, final double xSpeed, final double zRotation) {

    SmartDashboard.putNumber("Gyro", Robot.gyro.getAngle());
    SmartDashboard.putNumber("Gyro Ranged", getAngle());
    SmartDashboard.putNumber("Z Rotation", zRotation);
    SmartDashboard.putNumber("Kp", RobotMap.Kp);
    SmartDashboard.putNumber("last Heading", lastHeading);
    SmartDashboard.putString("Mode", mode);

    if (Math.abs(zRotation) > (RobotMap.threshold)) {
      // use manual drive
      mecanum_drive.driveCartesian(ySpeed, xSpeed, zRotation);
      // last heading angle updated
      lastHeading = getAngle();
      mode = "Manual";
    } else {
      SmartDashboard.putNumber("Auto Ouput", rotOutput);
      error = lastHeading - getAngle();
      rotOutput = RobotMap.Kp * error;
      mecanum_drive.driveCartesian(ySpeed * RobotMap.throttleCut, xSpeed * RobotMap.throttleCut,
          rotOutput * RobotMap.throttleCut);
      mode = "auto";
      Robot.timer.delay(RobotMap.delayTime);
    }
  }

  // calibrates then resets the gyro
  public static void resetGyro() {
    Robot.gyro.reset();
  }

  // makes gyro range (-360,360)
  public static double getAngle() {
    if (Robot.gyro.getAngle() > 360) {
      return (Robot.gyro.getAngle() % 360);
    } else if (Robot.gyro.getAngle() < -360) {
      return (Robot.gyro.getAngle() % 360);
    } else {
      return Robot.gyro.getAngle();
    }
  }

  // stops drive motors
  public void stop() {
    mecanum_drive.driveCartesian(0, 0, 0);
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveController());
  }
}
