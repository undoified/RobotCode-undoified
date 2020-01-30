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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation;

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

  private static Double wheelDiameter = 6.0/12.0;

  //Create encoder objects
  private Encoder driveEncoder = new Encoder(RobotMap.encoderPorts[0], RobotMap.encoderPorts[1], false, Encoder.EncodingType.k4X);

  //Create motor controller objects
  private VictorSP leftFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
  private PWMVictorSPX leftBackMotor = new PWMVictorSPX(RobotMap.leftBackMotor);
  private VictorSP rightFrontMotor = new VictorSP(RobotMap.rightFrontMotor);
  private PWMVictorSPX rightBackMotor = new PWMVictorSPX(RobotMap.rightBackMotor);

  private MecanumDrive mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

  public DriveTrain() {
    //The distance per pulse used here is for the am-3749 encoder.
    driveEncoder.setDistancePerPulse(wheelDiameter*3.14/1024);
  }


  public void moveMecanumDrive(double movementSpeed, double strafeSpeed, double turningSpeed){
    mecanumDrive.driveCartesian(-strafeSpeed, movementSpeed, turningSpeed);
  }

  public void driveToPoint(String direction, double distance, double speed) {
    switch (direction) {
      case "forward":
        while (driveEncoder.getDistance() < distance) {
          mecanumDrive.drivePolar(speed, 0, speed);
        }
        driveEncoder.reset();
        break;

      case "backward":
        while (driveEncoder.getDistance() < distance) {
          mecanumDrive.drivePolar(speed, 180, speed);
        }
        driveEncoder.reset();
        break;

      case "left":
        while (driveEncoder.getDistance() < distance) {
          mecanumDrive.drivePolar(speed, 90, speed);
        }
        driveEncoder.reset();
        break;

      case "right":
        while (driveEncoder.getDistance() < distance) {
          mecanumDrive.drivePolar(speed, -90, speed);
        }
        driveEncoder.reset();
        break;

      default:
        DriverStation.reportError("That is not a valid direction! Use 'forward', 'backward', 'left', or 'right'.", true);
        break;
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveRobot());
  }
}
