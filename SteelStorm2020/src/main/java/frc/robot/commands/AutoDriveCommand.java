/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Robot.autoDrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.AutonDriveSubsystem;
import frc.robot.subsystems.DriveTrainSubsytem;
import frc.robot.Robot;

public class AutoDriveCommand extends Command {

  public AutoDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.autoDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    autoDrive.setUpSystem();
    System.out.println("________________AutonCommand INTit______________");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    autoDrive.autonTimerMoveForward();
    System.out.println("^^^^^^^^^^^^auto execute^^^^^^^^^^^^^^");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return autoDrive.isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("******Auto Drive end()*******");
    autoDrive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("-------------Auto Drive intereupt--------------");
    autoDrive.stop();
  }
}
