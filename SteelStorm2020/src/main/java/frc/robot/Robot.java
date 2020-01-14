/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static frc.robot.OI.operator;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallCollectSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.DriveTrainSubsytem;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static Subsystem driveTrain;
  public static Subsystem ballCollect;
  public static Subsystem ballIntake;
  public static OI m_oi;
  //vision thread for cameras
  //Thread visionThread;
  Command autonCommand;
  //object to asign auton mode to
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  //setup gyro
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  //normla use timer
  public static Timer timer = new Timer();
  //for testing mode
  public static Timer testTimer = new Timer();
  //test variable for starting clock
  public boolean testTimerSet = false;
  //for closing pneumatic in test
  public boolean pneumaticTestClose = true;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    //calibrate gyro
    gyro.calibrate();

    //Instantiate all subsytems 
    driveTrain = new DriveTrainSubsytem();
    ballCollect = new BallCollectSubsystem();
    ballIntake = new BallIntakeSubsystem();
    m_oi = new OI();

    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(ballCollect);
    SmartDashboard.putData(ballIntake);

  
    //basically for auton "Defualt Auto" will be called since set as defualt, or if selected
    //and will call command "Example Command" for example here
   // m_chooser.setDefaultOption("Default Auto", new AutoDriveCommand());
   // m_chooser.addOption("Manual mode", new DriveController());
   // SmartDashboard.putData("Auto mode", m_chooser);

    //setup USB cameras:
    //Start camera server
    final CameraServer server = CameraServer.getInstance();
    //get usb cams
    final UsbCamera fCam = server.startAutomaticCapture("Front Cam", RobotMap.frontCam);
    final UsbCamera bCam = server.startAutomaticCapture("Back Cam", RobotMap.backCam);
    //set settings for cams
    if (fCam.setFPS(20) && bCam.setFPS(20) && fCam.setResolution(RobotMap.Img_Width, RobotMap.Img_Height) && bCam.setResolution(160, 120))
    {
      SmartDashboard.putBoolean("Camera Status", true);
    } else {
      SmartDashboard.putBoolean("Camera Status", false);
    }

/*
    //config vison thread
    visionThread = new Thread(() -> {

      //sets A sink for user code to accept video frames as OpenCV images
      CvSink cvSink = CameraServer.getInstance().getVideo(fCam);
      //A source that represents a video camera to put on dashboard driver
      CvSource cvSource = CameraServer.getInstance().putVideo("CV fCam", RobotMap.Img_Width, RobotMap.Img_Height);
      // Mat (matrix) for storing images to process, very memory expensive
      Mat frame = new Mat();
      //creating grip object
      GripPipeline pipeline = new GripPipeline();
      // lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
			  // Tell the CvSink to grab a frame from the camera and put it
				// in the source image. If there is an error notify the output.
				if (cvSink.grabFrame(frame) == 0) {
					// Send the output the error.
					cvSource.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
        }
        //calls grip pipeline to process image
				pipeline.process(frame);
				//displays number of contours found
        SmartDashboard.putNumber("number of contours", pipeline.findContoursOutput().size());
        // draw all contours (-1 = all)
        Imgproc.drawContours(frame, pipeline.findContoursOutput(), -1, new Scalar(0, 0, 255));
        // Give the output stream a new image to display
				cvSource.putFrame(frame);
			}

    });

    //start thread (must set Daemon before thread)
    try {
      visionThread.setDaemon(true);
      visionThread.start();
    } catch(Exception e){
     SmartDashboard.putBoolean("Vision Thread", false); 
    } */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
  
    //autonCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    /* //schedule the autonomous command (example)
    if (autonCommand != null) {
      autonCommand.start();
    } */

    timer.reset();
    timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (timer.get() <= RobotMap.driveTime){
      //((DriveTrainSubsytem) driveTrain).mecanumAngleDrive(0, RobotMap.autonSpeed, 0);
      ((DriveTrainSubsytem) driveTrain).mecanumDriveMethod(0, -RobotMap.autonSpeed, 0);
    } else if ((timer.get() <= RobotMap.driveTimeEnd) && (timer.get() > RobotMap.driveTimePause)){
      ((DriveTrainSubsytem) driveTrain).mecanumDriveMethod(0, RobotMap.autonSpeedEnd, 0);
    } else {
      //((DriveTrainSubsytem) driveTrain).mecanumAngleDrive(0, 0, 0);
      ((DriveTrainSubsytem) driveTrain).mecanumDriveMethod(0, 0, 0);
    }
  }


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
//    manualDriveCommand = new DriveController();

    if (autonCommand != null) {
     // autonCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  //tests
  @Override
  public void testPeriodic() {
    if (!testTimerSet){
      testSetup();
      //after timer starts
      //1. extends pneumatics (must be upt to presure first)
      ((BallCollectSubsystem) ballCollect).checkCollector(true, false, false);
      ((BallCollectSubsystem) ballCollect).setCollector();
    } else if (testTimer.get() <= RobotMap.motorTimeIn){
      //2. moves wheels in
      ((BallIntakeSubsystem) ballIntake).spinWheels(0, RobotMap.motorTestSpeed);
    } else if ((testTimer.get() >= RobotMap.motorTimePause) && (testTimer.get() <= RobotMap.motorTimeOut)){
      //3. moves wheels out
      ((BallIntakeSubsystem) ballIntake).spinWheels(RobotMap.motorTestSpeed, 0);
    } else if (testTimer.get() > (RobotMap.pneumaticRetractTime) && (pneumaticTestClose)){
      //4. retracts pneumatic
      ((BallCollectSubsystem) ballCollect).checkCollector(false, true, false);
      ((BallCollectSubsystem) ballCollect).setCollector();
      pneumaticTestClose = false;
    } else if (operator.getYButton()){
      //restarts system
      testTimerSet = false;
      pneumaticTestClose = true;
    } else {
      //stops wheels
      ((BallIntakeSubsystem) ballIntake).spinWheels(0, 0);
    }

  }

  public void testSetup(){
    testTimer.reset();
    testTimer.start();
    testTimerSet = true;
  }
}
