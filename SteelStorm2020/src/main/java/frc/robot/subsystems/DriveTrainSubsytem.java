/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
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

  //assigning motors to speed controls relative chanel
  public static SpeedController fLeftCim = new Talon(RobotMap.fLeftCim);
  public static SpeedController bLeftCim = new Talon(RobotMap.bLeftCim);
  public static SpeedController fRightCim = new Talon(RobotMap.fRightCim);
  public static SpeedController bRightCim = new Talon(RobotMap.bRightCim);

  //assigning speedcontroller groups for left and right
  public static SpeedControllerGroup left = new SpeedControllerGroup(fLeftCim, bLeftCim);
  public static SpeedControllerGroup right = new SpeedControllerGroup(fRightCim, bRightCim);

  //creating differential drive
  DifferentialDrive drive = new DifferentialDrive(left, right);

  //creating mecanum drive
  MecanumDrive mecanum_drive = new MecanumDrive(fLeftCim, bLeftCim, fRightCim, bRightCim);

  //runs method on startup (constructer method)
  public DriveTrainSubsytem(){
    drive.setSafetyEnabled(true);
    //not sure what these do
    drive.setExpiration(1);
    drive.setMaxOutput(1.0);
  }

  //tank drive method
  public void TankDrive(double leftAxis, double rightAxis, boolean button1, boolean button2){
    toggleButton(button1, button2);
    if (!toggle){
      drive.tankDrive(leftAxis, rightAxis);
    } else {
      drive.tankDrive(-leftAxis, -rightAxis);
    }
  }

  //arcadedrive method
  public void ArcadeDrive(double xSpeed, double zRotation, boolean buttonA, boolean buttonB){
    toggleButton(buttonA, buttonB);
    if (!toggle){
      drive.arcadeDrive(xSpeed, zRotation);
    } else {
      drive.arcadeDrive(-xSpeed, zRotation);
    }
  }

  public void MecanumDriveMethod(double ySpeed, double xSpeed, double zRotation){
    //can use gyro oriented drive with other version of method
    mecanum_drive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  //toggle method to reverse drive directions
  public void toggleButton(boolean buttonA, boolean buttonB){
    if (buttonA){
      toggle = true;
    } else if (buttonB){
      toggle = false;
    }
  }
  
  //stops drive motors
  public static void stop() {
		left.stopMotor();
		right.stopMotor();
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveController());
  }
}
