/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.RobotMap;
import frc.robot.commands.DriveRobot;

/**
 * DRIVE TRAIN
 * 
 * A basic mecanum drive subsytem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //private static final Double wheelDiameter = 6.0/12.0;

  //Create motor controller objects
  private WPI_VictorSPX leftMotor = new WPI_VictorSPX(RobotMap.c_leftMotor);
  private WPI_VictorSPX rightMotor = new WPI_VictorSPX(RobotMap.c_rightMotor);

  private DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public DriveTrain() {
  }


  public void moveArcadeDrive(double movementSpeed, double turningSpeed) {
    drive.arcadeDrive(movementSpeed, turningSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveRobot());
  }
}
