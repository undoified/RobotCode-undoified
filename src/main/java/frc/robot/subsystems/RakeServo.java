/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class RakeServo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Servo rakeServo = new Servo(RobotMap.rakeServo);
  //The current position, or stage, the rake is in.
  public int stage = 2;
  public int downAngle = 10;

  public RakeServo(){
    //Reset the stage when the robot is turned on.
    rakeServo.setAngle(40);
  }
  
  public void changeStage() {
    switch(stage) {
      case 1:
        while (downAngle <= 100){
          rakeServo.setAngle(downAngle);
          Timer.delay(0.09);
          SmartDashboard.putNumber("Down Angle", downAngle);
          downAngle = downAngle + 10;
        };
        downAngle = 10;
        break;
      case 2:
        rakeServo.setAngle(40);
        break;
      case 3:
        rakeServo.setAngle(0);
        break;
      default:
        //If the stage values are to big or small reset the values to a valid stage and do nothing.
        if (stage > 3) {
          stage = 3;
        }
        if (stage < 0) {
          stage = 0;
        }
        break;
    }
  }

  public double getArmAngle(){
    return rakeServo.getAngle();
  }

  public int getStage(){
    return stage;
  }

  public int getDownAngle(){
    return downAngle;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
