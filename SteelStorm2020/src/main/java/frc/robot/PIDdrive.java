/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Add your docs here.
 */
public class PIDdrive extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  //assigning motors to speed controls relative chanel
  public static SpeedController fLeftCim = new Talon(RobotMap.fLeftCim);
  public static SpeedController bLeftCim = new Talon(RobotMap.bLeftCim);
  public static SpeedController fRightCim = new Talon(RobotMap.fRightCim);
  public static SpeedController bRightCim = new Talon(RobotMap.bRightCim);

  //creating mecanum drive object
  MecanumDrive mecanum_drive = new MecanumDrive(fLeftCim, bLeftCim, fRightCim, bRightCim);
  
  //gyro setup
  Gyro gyro = new AnalogGyro(0);
  double pidOutput;

  public PIDdrive() {
    // Intert a subsystem name and PID values here
    super("PIDdrive", 0.3, 0, 0);
    //YES or NO -- test
    getPIDController().setContinuous(true);
    setInputRange(-180,180);
    setOutputRange(-1, 1);
    setAbsoluteTolerance(3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    output = pidOutput;
  }

  public void MecanumDrivePID(double ySpeed, double xSpeed, double zSpeed){
    if (Math.abs(zSpeed) > 0.1){
      //reset PID
      mecanum_drive.driveCartesian(ySpeed, xSpeed, zSpeed);
    } else {
      mecanum_drive.driveCartesian(ySpeed, xSpeed, pidOutput);
    }
  }
}
