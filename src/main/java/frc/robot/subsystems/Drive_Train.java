// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.util.Settings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive_Train extends SubsystemBase {
  /** Creates a new Drive_Train. */

  private CANSparkMax _fLMotor = new CANSparkMax(DrivetrainConstants.frontLeftMotor, MotorType.kBrushless);
  private CANSparkMax _fRMotor = new CANSparkMax(DrivetrainConstants.frontRightMotor, MotorType.kBrushless);
  private CANSparkMax _bLMotor = new CANSparkMax(DrivetrainConstants.rearLeftMotor, MotorType.kBrushless);
  private CANSparkMax _bRMotor = new CANSparkMax(DrivetrainConstants.rearRightMotor, MotorType.kBrushless); 

  private final MecanumDrive _drive = new MecanumDrive(_fLMotor, _bLMotor, _fRMotor, _bRMotor);

  private AHRS _gyro;
  
  private RelativeEncoder m_left_follower;
  private RelativeEncoder m_right_follower;

  private DifferentialDriveOdometry m_odometry;

  private Pose2d m_pose;

  private double _kP = LimelightConstants.kP;
  private double _kI = LimelightConstants.kI;
  private double _kD = LimelightConstants.kD;
  private double _kF = LimelightConstants.kF;

  private double _ksVolts = DrivetrainConstants.ksVolts;
  private double _kvVoltSecondsPerMeter = DrivetrainConstants.kvVoltSecondsPerMeter;
  private double _kaVoltSecondsSquaredPerMeter = DrivetrainConstants.kaVoltSecondsSquaredPerMeter;

  private double _kachunkPower = DrivetrainConstants.kachunkPower;
  private double _kachunkTime = DrivetrainConstants.kachunkTime;
  private double _driveRightkP = DrivetrainConstants.driveRightkP;
  private double _nerf = 1.0;

  public Drive_Train(AHRS gyro) {

    _gyro = gyro;
    // _gyro.setYawAxis(IMUAxis.kY);
    _gyro.reset();

    _fLMotor.restoreFactoryDefaults();
    _fRMotor.restoreFactoryDefaults();
    _bRMotor.restoreFactoryDefaults();
    _bLMotor.restoreFactoryDefaults();

    _fRMotor.setInverted(true);
    _fLMotor.setInverted(false);
    _bRMotor.setInverted(true);
    _bLMotor.setInverted(false);

    enableOpenLoopRampRate(true);

    m_left_follower = _fLMotor.getEncoder();
    m_right_follower = _fRMotor.getEncoder();

    m_left_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);
    m_right_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);

    encoderReset();
    
    m_odometry = new DifferentialDriveOdometry(_gyro.getRotation2d(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

  }

  public void enableOpenLoopRampRate(boolean enable) {
    double rampRate = (enable ? DrivetrainConstants.rampRate : 0.0);

    _fLMotor.setOpenLoopRampRate(rampRate);
    _fRMotor.setOpenLoopRampRate(rampRate);
    _bLMotor.setOpenLoopRampRate(rampRate);
    _bRMotor.setOpenLoopRampRate(rampRate);
  }

  public void teleopDrive(Joystick driveControl){
    double ySpeed = applyDeadband(driveControl.getRawAxis(JoystickConstants.LEFT_STICK_Y));
    double xSpeed = applyDeadband(driveControl.getRawAxis(JoystickConstants.LEFT_STICK_X));
    double zRotation = applyDeadband(driveControl.getRawAxis(JoystickConstants.RIGHT_STICK_X));

    _drive.driveCartesian(-ySpeed, xSpeed, zRotation, _gyro.getAngle());

    SmartDashboard.putNumber("Gyro.Angle", _gyro.getAngle());
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < DrivetrainConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(.1, value)) / (1 - DrivetrainConstants.deadband);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    _drive.driveCartesian(ySpeed, -xSpeed, zRotation);
  }

  public void encoderReset() {
    m_right_follower.setPosition(0.0);
    m_left_follower.setPosition(0.0);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_left_follower.getVelocity() / 60, m_right_follower.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pose, _gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _fLMotor.setVoltage(leftVolts);
    _fRMotor.setVoltage(rightVolts);
    _bRMotor.setVoltage(rightVolts);
    _bLMotor.setVoltage(leftVolts);
  }

  public void setkP(double kP){
    _kP = kP;
  }

  public void setkI(double kI){
    _kI = kI;
  }

  public void setkD(double kD){
    _kD = kD;
  }

  public void setkF(double kF){
    _kF = kF;
  }

  public double getkP(){
    return _kP;
  }

  public double getkI(){
    return _kI;
  }

  public double getkD(){
    return _kD;
  }

  public double getkF(){
    return _kF;
  }

  public double getkaVoltSecondsSquaredPerMeter() {
    return _kaVoltSecondsSquaredPerMeter;
  }

  public double getkvVoltSecondsPerMeter() {
    return _kvVoltSecondsPerMeter;
  }

  public double getksVolts(){
    return _ksVolts;
  }

  public void setkaVoltSecondsSquaredPerMeter(double kaVoltSecondsSquaredPerMeter) {
    _kaVoltSecondsSquaredPerMeter = kaVoltSecondsSquaredPerMeter;
  }

  public void setkvVoltSecondsPerMeter(double kvVoltSecondsPerMeter) {
    _kvVoltSecondsPerMeter = kvVoltSecondsPerMeter;
  }

  public void setksVolts(double ksVolts){
    _ksVolts = ksVolts;
  }

  public void setKachunkPower(double power) {
    _kachunkPower = power;
  }

  public void setKachunkTime(double seconds) {
    _kachunkTime = seconds;
  }

  public double getKachunkPower() {
    return _kachunkPower;
  }

  public double getKachunkTime() {
    return _kachunkTime;
  }

  public void setDriveRightkP(double kP) {
    _driveRightkP = kP;
  }

  public double getDriveRightkP() {
    return _driveRightkP;
  }

  public void setNerf(double nerf) {
    if(nerf < 0) nerf = Math.abs(nerf);
    if(nerf > 1.0) nerf = 1.0;
    _nerf = nerf;
  }

  public double getNerf() {
    return _nerf;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pose = m_odometry.update(_gyro.getRotation2d(), m_left_follower.getPosition(), m_right_follower.getPosition());
    
    setkP(Settings.getLiveDouble("Limelight", "kP", LimelightConstants.kP));
    setkI(Settings.getLiveDouble("Limelight", "kI", LimelightConstants.kI));
    setkD(Settings.getLiveDouble("Limelight", "kD", LimelightConstants.kD));
    
    setksVolts(Settings.getLiveDouble("DriveTrain", "ksVolts", DrivetrainConstants.ksVolts));
    setkvVoltSecondsPerMeter(Settings.getLiveDouble("DriveTrain", "kvVoltSecondsPerMeter", DrivetrainConstants.kvVoltSecondsPerMeter));
    setkaVoltSecondsSquaredPerMeter(Settings.getLiveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
  }
}
