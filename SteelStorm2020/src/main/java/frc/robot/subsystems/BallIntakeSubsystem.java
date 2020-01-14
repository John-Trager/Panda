/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.BallIntakeCommand;

/**
 * Add your docs here.
 */
public class BallIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // assigning motors to speed controls to relative chanel
  public  static SpeedController leftMotor = new Talon(RobotMap.leftBallMotor);
  public static SpeedController rightMotor = new Talon(RobotMap.rightBallMotor);
  
  public void spinWheels(double leftTrigger, double rightTrigger){
    if (Math.abs(leftTrigger) > 0.05){
      leftMotor.set(leftTrigger*0.9);
      rightMotor.set(-leftTrigger*0.9);
    } else if (Math.abs(rightTrigger)>0.05){
      leftMotor.set(-rightTrigger*0.9);
      rightMotor.set(rightTrigger*0.9);
    } else {
      leftMotor.set(0);
      rightMotor.set(0);
    }
    SmartDashboard.putNumber("rBallmSpeed", leftMotor.get());
    SmartDashboard.putNumber("lBallmSpeed", leftMotor.get());
  }

  public void stop(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new BallIntakeCommand());
  }
}
