/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.RobotMap;

/**
 * DRIVE TRAIN
 * 
 * A basic arcade drive subsytem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Create motor controller objects
  private VictorSP leftMaster = new VictorSP(RobotMap.leftMasterMotor);
  private VictorSP leftSlave = new VictorSP(RobotMap.leftSlaveMotor);
  private VictorSP rightMaster = new VictorSP(RobotMap.rightMasterMotor);
  private VictorSP rightSlave = new VictorSP(RobotMap.rightSlaveMotor);


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
