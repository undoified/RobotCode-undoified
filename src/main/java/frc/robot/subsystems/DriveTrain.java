/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.commands.DriveRobot;

/**
 * DRIVE TRAIN
 * 
 * A basic arcade drive subsytem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //Create motor controller objects
  private VictorSP leftFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
  private VictorSP leftBackMotor = new VictorSP(RobotMap.leftBackMotor);
  private VictorSP rightFrontMotor = new VictorSP(RobotMap.rightFrontMotor);
  private VictorSP rightBackMotor = new VictorSP(RobotMap.rightBackMotor);

  private MecanumDrive mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

  public void moveMecanumDrive(double movementSpeed, double strafeSpeed, double turningSpeed){
    mecanumDrive.driveCartesian(movementSpeed, strafeSpeed, turningSpeed);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveRobot());
  }
}
