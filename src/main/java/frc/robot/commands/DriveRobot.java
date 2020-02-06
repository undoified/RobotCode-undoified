/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * ARCADE DRIVE COMMAND
 * 
 * A basic arcade drive command.
 */
public class DriveRobot extends Command {
  public DriveRobot() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //double joystickMovementSpeed = Robot.oi.joystick.getY();
    //double joystickStrafeSpeed = Robot.oi.joystick.getX();
    //This is how the orginal way was, but it caused the robot 
    //to move backwards when the joystick was pushed forward
    //In wpilib the yspeed is the robot's speed along the Yaxis, with right positive
    //the xspeed is the robot's speed along the Xaxis, with forward positive
    //We switched the X and the Y so that when the joystick is forward the robot goes forward
    double joystickMovementSpeed = Robot.oi.joystick.getX();
    double joystickStrafeSpeed = Robot.oi.joystick.getY();
    double joystickTurnSpeed = Robot.oi.joystick.getZ();
    double deadzone = 0.2;
    if (Math.abs(Robot.oi.joystick.getX()) < deadzone) {
      joystickMovementSpeed = 0;
    }
    if (Math.abs(Robot.oi.joystick.getY()) < deadzone) {
      joystickStrafeSpeed = 0;
    }
    if (Math.abs(Robot.oi.joystick.getZ()) < deadzone) {
      joystickTurnSpeed = 0;
    }
    Robot.driveTrain.moveMecanumDrive(joystickStrafeSpeed, joystickMovementSpeed, joystickTurnSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
