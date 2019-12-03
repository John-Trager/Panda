/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  
  //4 main cim motors of drive train PWM
  public static int fLeftCim = 1;
  public static int fRightCim = 3;
  public static int bLeftCim = 2;
  public static int bRightCim = 4;
  //lift motors
  public static int lLiftMotor = 2;
  public static int rLiftMotor = 3;
  //camera sources
  public static int frontCam = 0;
  public static int backCam = 1;
  //lift encoder mxp dio ports
  public static final int encoderPortA = 8;
  public static final int encoderPortB = 9;
  //camera settins
  public static int Img_Width = 120;
  public static int Img_Height = 160;
  //drive PID tuning
  public static double threshold = 0.1;
  public static double Kp = 0.03;
  //analog port
  public static int mb1013Port = 0;
  //timer for auton moving
  public static double driveTime = 3.0;
  public static double autonSpeed = 0.3;
  //ratio voltage to inches for ultra-sonic sensor
  public static double VoltToInches = 1.0;
  //throttle limit cut
  public static double throttleCut = 0.6;
  //time delay for P controllers
  public static double delayTime = 0.005;
}
