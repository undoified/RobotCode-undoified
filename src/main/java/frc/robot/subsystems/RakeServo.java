/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class RakeServo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Servo rakeServo = new Servo(RobotMap.rakeServo);
  //The amount added to the current postition of the servo
  //private double rakeServoDistance = 70;

  public RakeServo(){
    //rakeServo.setAngle(80);
  }

  public void rakeBottom(){
    //rakeServo.setAngle(rakeServo.getAngle() + rakeServoDistance);
    rakeServo.setAngle(0);
  }

  public void rakeCarry(){
    //rakeServo.setAngle(rakeServo.getAngle() - rakeServoDistance);
    rakeServo.setAngle(60);
  }

  public void rakeFling(){
    rakeServo.setAngle(100);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
