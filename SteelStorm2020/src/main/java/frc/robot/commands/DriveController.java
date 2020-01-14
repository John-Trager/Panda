/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.OI.driver;
import static frc.robot.Robot.driveTrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsytem;

public class DriveController extends Command {
  public DriveController() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //System.out.println("%%%%%%%%%%%%%%%Manual DRIVE%%%%%%%%%%%%%%%%%");
    ((DriveTrainSubsytem) driveTrain).mecanumDriveMethod(-driver.getX(Hand.kRight), -driver.getY(Hand.kRight), driver.getX(Hand.kLeft));
    //((DriveTrainSubsytem) driveTrain).mecanumAngleDrive(-driver.getX(Hand.kRight), -driver.getY(Hand.kRight), driver.getX(Hand.kLeft));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //stops all motors
    ((DriveTrainSubsytem) driveTrain).stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
