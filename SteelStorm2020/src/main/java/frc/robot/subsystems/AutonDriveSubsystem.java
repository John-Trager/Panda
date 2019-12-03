/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutoDriveCommand;

/**
 * Add your docs here.
 */
public class AutonDriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //ultra-sonic sensor
  public static final AnalogInput mb1013 = new AnalogInput(RobotMap.mb1013Port);
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
  // creating timer
  public final static Timer timer = new Timer();
  // assigning motors to speed controls relative chanel
  public static SpeedController fLeftCim = new Talon(RobotMap.fLeftCim);
  public static SpeedController bLeftCim = new Talon(RobotMap.bLeftCim);
  public static SpeedController fRightCim = new Talon(RobotMap.fRightCim);
  public static SpeedController bRightCim = new Talon(RobotMap.bRightCim);

  public static MecanumDrive mecanum_drive = new MecanumDrive(fLeftCim, bLeftCim, fRightCim, bRightCim);

  public AutonDriveSubsystem(){
    System.out.println("AutonDriveSubsystem Started");
  }

  // calibrates then resets the gyro
  public static void resetGyro() {
    Robot.gyro.reset();
  }

  // gets angle for auton starts timer
  public static void setAngle() {
    // timer.reset();
    timer.start();
    setAngle = Robot.gyro.getAngle();
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

  // return distance
  public double getDistance() {
    double distance = mb1013.getVoltage() * RobotMap.VoltToInches;
    SmartDashboard.putNumber("Distance(IN)", distance);
    return distance;
  }

  public void autonTimerMoveForward() {
    lastHeading = getAngle();
    error = lastHeading - setAngle;
    rotOutput = RobotMap.Kp * error;
    if (timer.get() <= RobotMap.driveTime) {
      autoDriveState = "IN";
      mecanum_drive.driveCartesian(RobotMap.autonSpeed, 0, rotOutput * 0.7);
    } else {
      autoDriveState = "OUT";
      mecanum_drive.driveCartesian(0, 0, 0);
    }
    SmartDashboard.putString("AUTO DRIVE", autoDriveState);
  }

  public void autonDistanceMoveForward() {
    lastHeading = getAngle();
    error = lastHeading - setAngle;
    rotOutput = RobotMap.Kp * error;
    if (getDistance() >= 24.0) {
      mecanum_drive.driveCartesian(RobotMap.autonSpeed, 0, rotOutput * 0.7);
    } else {
      mecanum_drive.driveCartesian(0, 0, 0);
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new AutoDriveCommand());
  
  }
}
