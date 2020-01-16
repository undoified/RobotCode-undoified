/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PWMVictorSPX;
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
  private PWMVictorSPX leftBackMotor = new PWMVictorSPX(RobotMap.leftBackMotor);
  private VictorSP rightFrontMotor = new VictorSP(RobotMap.rightFrontMotor);
  private PWMVictorSPX rightBackMotor = new PWMVictorSPX(RobotMap.rightBackMotor);

  //This is not the order from the documetation, 
  //but through trial and error this is the only way that it works
  private MecanumDrive mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

  public DriveTrain() {
  
  }


  public void moveMecanumDrive(double movementSpeed, double strafeSpeed, double turningSpeed){
    mecanumDrive.driveCartesian(-strafeSpeed, movementSpeed, turningSpeed);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveRobot());
  }
}
