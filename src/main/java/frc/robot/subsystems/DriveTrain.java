/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation;
import com.analog.adis16470.frc.ADIS16470_IMU;

import frc.robot.RobotMap;
//import frc.robot.Robot;
import frc.robot.commands.DriveRobot;

/**
 * DRIVE TRAIN
 * 
 * A basic mecanum drive subsytem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final Double wheelDiameter = 6.0/12.0;

  //Create IMU object
  public ADIS16470_IMU imu = new ADIS16470_IMU();

  //Create encoder objects
  public Encoder driveEncoder = new Encoder(RobotMap.driveEncoderPorts[0], RobotMap.driveEncoderPorts[1], false, Encoder.EncodingType.k4X);

  //Create motor controller objects
  private WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(RobotMap.leftFrontMotor);
  private WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(RobotMap.leftBackMotor);
  private WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RobotMap.rightFrontMotor);
  private WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RobotMap.rightBackMotor);

  private MecanumDrive mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

  public DriveTrain() {
    //The distance per pulse used here is for the am-3749 encoder.
    driveEncoder.setDistancePerPulse(wheelDiameter*3.14/1024);
  }


  public void moveMecanumDrive(double movementSpeed, double strafeSpeed, double turningSpeed) {
    mecanumDrive.driveCartesian(-strafeSpeed, movementSpeed, turningSpeed);
  }

  public void driveToPoint(String direction, double distance, double speed) {
    driveEncoder.reset();
    switch (direction) {
      //The left and right speed values may be incorrect
      case "forward":
        while (Math.abs(driveEncoder.getDistance()) < distance) {
          moveMecanumDrive(speed, 0, 0);
        }
        driveEncoder.reset();
        break;

      case "backward":
        while (Math.abs(driveEncoder.getDistance()) < distance) {
          moveMecanumDrive(-speed, 0, 0);
        }
        driveEncoder.reset();
        break;

      case "left":
        while (Math.abs(driveEncoder.getDistance()) < distance) {
          moveMecanumDrive(0, speed, 0);
        }
        driveEncoder.reset();
        break;

      case "right":
        while (Math.abs(driveEncoder.getDistance()) < distance) {
          moveMecanumDrive(0, -speed, 0);
        }
        driveEncoder.reset();
        break;

      default:
        DriverStation.reportError("That is not a valid direction! Use 'forward', 'backward', 'left', or 'right'.", true);
        break;
    }
  }
  //This (probably) doesn't work
  public void turnRobot(String direction, double angle, double speed) {
    imu.reset();
    switch (direction) {
      case "left":
          while (angle < imu.getAngle()) {
            moveMecanumDrive(0, 0, speed);
          }
        break;

      case "right":
        while (angle < imu.getAngle()) {
          moveMecanumDrive(0, 0, -speed);
        }
        break;

      default:
        DriverStation.reportError("That is not a valid direction! Use left', or 'right'.", true);
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
