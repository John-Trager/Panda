/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.BallCollect;

/**
 * Add your docs here.
 */
public class BallCollectSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //Instantiate pneumatics
  public DoubleSolenoid collectorRight = new DoubleSolenoid(0, 1);
  public DoubleSolenoid collectorLeft = new DoubleSolenoid(2, 3);

  // 1 is in, 2 is extend, 3 or else is off CHANGE IF WIRED DIFFERENT
  public int extendSolonoid = 2;

  public BallCollectSubsystem() {
    // may need to disable during P controller and for auton
    //drive.setSafetyEnabled(true);
    System.out.println("BallCollectSystem Started");
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new BallCollect());
  }
  
  public void checkCollector(boolean bBoolean, boolean aBoolean, boolean yBoolean){
    if (bBoolean == true){
      extendSolonoid = 1;
    } else if (aBoolean == true){
      extendSolonoid = 2;
    } else if (yBoolean == true){
      extendSolonoid = 0;
    }
  }

  public void setCollector(){
    if (extendSolonoid == 1){
      collectorLeft.set(DoubleSolenoid.Value.kForward);
      collectorRight.set(DoubleSolenoid.Value.kForward);
    } else if (extendSolonoid == 2){
      collectorLeft.set(DoubleSolenoid.Value.kReverse);
      collectorRight.set(DoubleSolenoid.Value.kReverse);
    } else {
      collectorLeft.set(DoubleSolenoid.Value.kOff);
      collectorRight.set(DoubleSolenoid.Value.kOff);

    }
  }

  public void stop(){
    //shuts power off to solenoids
    collectorLeft.set(DoubleSolenoid.Value.kOff);
    collectorRight.set(DoubleSolenoid.Value.kOff);
  }


}
