// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive_Train extends SubsystemBase {
  /** Creates a new Drive_Train. */
  public Drive_Train(ADIS16470_IMU gyro) {

    _gyro = gyro;
    _gyro.setYawAxis(IMUAxis.kY);
    _gyro.reset();

    fLMotor.restoreFactoryDefaults();
    fRMotor.restoreFactoryDefaults();
    bRMotor.restoreFactoryDefaults();
    bLMotor.restoreFactoryDefaults();

    fRMotor.setInverted(true);
    fLMotor.setInverted(false);
    bRMotor.setInverted(true);
    bLMotor.setInverted(false);
  }

  private CANSparkMax fLMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax fRMotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax bLMotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax bRMotor = new CANSparkMax(4, MotorType.kBrushless); 

  private final MecanumDrive _drive = new MecanumDrive(fLMotor, bLMotor, fRMotor, bRMotor);

  private ADIS16470_IMU _gyro;

  public void teleopDrive(Joystick driveControl){
    double ySpeed = applyDeadband(driveControl.getRawAxis(1));
    double xSpeed = applyDeadband(driveControl.getRawAxis(0));
    double zRotation = applyDeadband(driveControl.getRawAxis(4));

    _drive.driveCartesian(-ySpeed, xSpeed, zRotation, _gyro.getAngle());
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < .1)
      return 0.0;
    else
      return (value - Math.copySign(.1, value)) / (1 - .1);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
