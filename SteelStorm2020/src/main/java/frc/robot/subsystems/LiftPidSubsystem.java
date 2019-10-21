/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.LiftController;

/**
 * Add your docs here.
 */
public class LiftPidSubsystem extends PIDSubsystem {
  /**
   * Add your docs here.
   */

  private int target = 2530;
  //config motor contollers
  public static TalonSRX lLiftMotor = new TalonSRX(RobotMap.lLiftMotor);
  public static TalonSRX rLiftMotor = new TalonSRX(RobotMap.rLiftMotor);

  //would also config encoder here if not talon



  public LiftPidSubsystem() {
    // Intert a subsystem name and PID values here
    super("LiftPidSubsystem", 0.0001, 0.0, 0.0);
    
    setPercentTolerance(3);
    //getPIDController().setContinuous(false);
    setOutputRange(0, 1.0);
    setSetpoint(target);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public void startLift(boolean buttonA){
    if (buttonA){
      enable();
    }
  }

  public void stopLift(boolean buttonB){
    if (buttonB){
      disable();
    }
  }

  public void stop(){
    disable();
    lLiftMotor.set(ControlMode.PercentOutput, 0);
    rLiftMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LiftController());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return lLiftMotor.getSelectedSensorPosition();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    lLiftMotor.set(ControlMode.PercentOutput, output);
    rLiftMotor.set(ControlMode.PercentOutput, output);
  }
}
