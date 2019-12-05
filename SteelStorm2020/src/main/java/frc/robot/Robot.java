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
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.BallCollect;
import frc.robot.commands.DriveController;
import frc.robot.subsystems.AutonDriveSubsystem;
import frc.robot.subsystems.BallCollectSubsystem;
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
  public static AutonDriveSubsystem autoDrive;
  public static OI m_oi;
  //vision thread for cameras
  //Thread visionThread;
  Command autonCommand;
  Command manualDriveCommand;
  //object to asign auton mode to
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  //setup gyro
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public static Timer timer = new Timer();


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    //Instantiate all subsytems 
    //public static BallCollectSubsystem ballCollect = new BallCollectSubsystem();
    driveTrain = new DriveTrainSubsytem();
   // autoDrive = new AutonDriveSubsystem();    
    m_oi = new OI();

    SmartDashboard.putData(driveTrain);
    //SmartDashboard.putData(autoDrive);
  
    //basically for auton "Defualt Auto" will be called since set as defualt, or if selected
    //and will call command "Example Command" for example here
   // m_chooser.setDefaultOption("Default Auto", new AutoDriveCommand());
   // m_chooser.addOption("Manual mode", new DriveController());
   // SmartDashboard.putData("Auto mode", m_chooser);
  
    //calibrate gyro
    gyro.calibrate();

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

    // schedule the autonomous command (example)
    if (autonCommand != null) {
      //autonCommand.start();
    }
    timer.reset();
    timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //Scheduler.getInstance().run();
    if (timer.get() <= 3.0){
      ((DriveTrainSubsytem) driveTrain).mecanumAngleDrive(0, 0.3, 0);
    } else {
      ((DriveTrainSubsytem) driveTrain).mecanumAngleDrive(0, 0, 0);
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
   // if (manualDriveCommand != null){
   //   manualDriveCommand.start();
   // }
    
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
  @Override
  public void testPeriodic() {

  }
}
