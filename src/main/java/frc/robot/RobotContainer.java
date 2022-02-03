// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterV2.COLOR;
import frc.robot.util.LIDARLiteV3;
import frc.robot.util.Limelight;
import frc.robot.util.Settings;
import frc.robot.util.TrajectoryCache;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
  private final Drive_Train _drive_Train = new Drive_Train(_gyro);
  private final Joystick _driver = new Joystick(0);
  private final Hopper _hopper = new Hopper();
  private final Intake _intake = new Intake();
  private final Limelight _limelight = new Limelight();
  private final ShooterV2 _shooter = new ShooterV2();
  private final LIDARLiteV3 _lidar = new LIDARLiteV3(0,0);

  private SendableChooser<String> m_ChallengeChooser = new SendableChooser<String>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    loadSettings();

    _drive_Train.setDefaultCommand(new RobotDrive(_drive_Train, _driver));

    loadChallengeChooser();
  }

  private void loadChallengeChooser() {
    TrajectoryCache.clear();
    m_ChallengeChooser = new SendableChooser<>();
    m_ChallengeChooser.addOption("Galactic Search", "Galactic Search");

    cacheTrajectory("AutoNav - Barrel Racing", "Paths/output/AutoNav--Barrel_Racing.wpilib.json");
    m_ChallengeChooser.addOption("AutoNav - Bounce", "AutoNav - Bounce");
    cacheTrajectory("AutoNav - Slalom", "Paths/output/AutoNav--Slalom.wpilib.json");

    cacheTrajectory("GS A Blue", "Paths/output/GS_A--Blue.wpilib.json");
    cacheTrajectory("GS A Red", "Paths/output/GS_A--Red.wpilib.json");
    cacheTrajectory("GS B Blue", "Paths/output/GS_B--Blue.wpilib.json");
    cacheTrajectory("GS B Red", "Paths/output/GS_B--Red.wpilib.json");

    cacheTrajectory("AutoNav--Bounce0", "Paths/output/AutoNav--Bounce0.wpilib.json");
    cacheTrajectory("AutoNav--Bounce1", "Paths/output/AutoNav--Bounce1.wpilib.json");
    cacheTrajectory("AutoNav--Bounce2", "Paths/output/AutoNav--Bounce2.wpilib.json");
    cacheTrajectory("AutoNav--Bounce3", "Paths/output/AutoNav--Bounce3.wpilib.json");

    cacheTrajectory("Test-Straight", "Paths/output/test-straight.wpilib.json");
    cacheTrajectory("Test-Turn", "Paths/output/test-turn.wpilib.json");
    cacheTrajectory("Test-Turn2", "Paths/output/test-turn2.wpilib.json");
    cacheTrajectory("Test-Curve", "Paths/output/test-curve.wpilib.json");
    cacheTrajectory("Test-StraightReverse", "Paths/output/test-straightreverse.wpilib.json");
    
    m_ChallengeChooser.addOption("Test-Group", "Test-Group");

    SmartDashboard.putData("Challenge Chooser", m_ChallengeChooser);
  }

  private void cacheTrajectory(String key, String trajectoryJson) {
    m_ChallengeChooser.addOption(key, key);
    TrajectoryCache.add(key, trajectoryJson);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //OPERATOR
    // // new JoystickButton(m_operator, JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(m_intake)); 
    // // new JoystickButton(m_operator, JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(m_intake)); 

    // new JoystickButton(m_operator, JoystickConstants.BUMPER_RIGHT).whileHeld(new HopperGo(m_hopper));
    // new JoystickButton(m_operator, JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(m_hopper));

   
    // new JoystickButton(m_operator, JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> m_limelight.toggleBypass()));

    //DRIVER
    new JoystickButton(_driver, JoystickConstants.LOGO_LEFT).whenPressed(new InstantCommand(() -> _gyro.reset()));

    //the buttons below are generally for testing purposes only
    new JoystickButton(_driver, JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(_intake)); 
    new JoystickButton(_driver, JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(_intake)); 

    new JoystickButton(_driver, JoystickConstants.BUMPER_RIGHT).whileHeld(new HopperIdle(_hopper));
    new JoystickButton(_driver, JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(_hopper));

    new JoystickButton(_driver, Constants.JoystickConstants.A).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Green)); 
    new JoystickButton(_driver, Constants.JoystickConstants.Y).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Yellow));
    new JoystickButton(_driver, Constants.JoystickConstants.X).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Blue));
    //new JoystickButton(_driver, Constants.JoystickConstants.B).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Red));
    new JoystickButton(_driver, Constants.JoystickConstants.B).whenReleased(new AutonIntake(_intake, _drive_Train, "Test-Curve"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    String challenge = m_ChallengeChooser.getSelected(); 

    Trajectory trajectory;
    if (challenge == "Galactic Search") {
      return new GalacticSearch(_drive_Train, _intake, _gyro, _lidar);
    } else if (challenge == "AutoNav - Bounce") {
      return new BounceSequence(_drive_Train);
    } else {
      trajectory = TrajectoryCache.get(challenge);
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        _drive_Train::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
          DrivetrainConstants.kvVoltSecondsPerMeter,
          DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        DrivetrainConstants.kDriveKinematics,
        _drive_Train::getWheelSpeeds,
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
        _drive_Train::tankDriveVolts,
        _drive_Train);
     // Reset odometry to the starting pose of the trajectory.
     _drive_Train.resetOdometry(trajectory.getInitialPose());

     // Run path following command, then stop at the end.
     return ramseteCommand.andThen(() -> _drive_Train.tankDriveVolts(0, 0));
  }

  public void loadSettings(){
    _intake.setPower(Settings.loadDouble("Intake", "Power", IntakeConstants.defaultPower));
    _hopper.setPower(Settings.loadDouble("Hopper", "Power", HopperConstants.defaultPower));
    _hopper.setFeederPower(Settings.getLiveDouble("Hopper", "FeederPower", HopperConstants.hopperFeederPower));
    _shooter.setkP(Settings.loadDouble("Shooter", "kF", ShooterConstants.defaultkF));
    _shooter.setkF(Settings.loadDouble("Shooter", "kP", ShooterConstants.defaultkP));
    _shooter.setRedVelocity(Settings.loadDouble("Shooter", "RedVelocity", ShooterConstants.redVelocity));
    _shooter.setBlueVelocity(Settings.loadDouble("Shooter", "BlueVelocity", ShooterConstants.blueVelocity));
    _shooter.setYellowVelocity(Settings.loadDouble("Shooter", "YellowVelocity", ShooterConstants.yellowVelocity));
    _shooter.setGreenVelocity(Settings.loadDouble("Shooter", "GreenVelocity", ShooterConstants.greenVelocity));
    _drive_Train.setkP(Settings.loadDouble("Limelight", "kP", LimelightConstants.kP));
    _drive_Train.setkI(Settings.loadDouble("Limelight", "kI", LimelightConstants.kI));
    _drive_Train.setkD(Settings.loadDouble("Limelight", "kD", LimelightConstants.kD));
    _drive_Train.setksVolts(Settings.loadDouble("DriveTrain", "ksVolts", DrivetrainConstants.ksVolts));
    _drive_Train.setkvVoltSecondsPerMeter(Settings.loadDouble("DriveTrain", "kvVoltSecondsPerMeter", DrivetrainConstants.kvVoltSecondsPerMeter));
    _drive_Train.setkaVoltSecondsSquaredPerMeter(Settings.loadDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
  }

  public void saveSettings(){
    Settings.saveDouble("Intake", "Power", _intake.getPower());
    Settings.saveDouble("Hopper", "Power", _hopper.getPower());
    Settings.saveDouble("Hopper", "FeederPower", _hopper.getFeederPower());
    Settings.saveDouble("Shooter", "kF", _shooter.getkF());
    Settings.saveDouble("Shooter", "kP", _shooter.getkP());
    Settings.saveDouble("Shooter", "RedVelocity", _shooter.getRedVelocity());
    Settings.saveDouble("Shooter", "BlueVelocity", _shooter.getBlueVelocity());
    Settings.saveDouble("Shooter", "YellowVelocity", _shooter.getYellowVelocity());
    Settings.saveDouble("Shooter", "GreenVelocity", _shooter.getGreenVelocity());
    Settings.saveDouble("Limelight", "kP", _drive_Train.getkP());
    Settings.saveDouble("Limelight", "kI", _drive_Train.getkI());
    Settings.saveDouble("Limelight", "kD", _drive_Train.getkD());
    Settings.saveDouble("DriveTrain", "ksVolts", _drive_Train.getksVolts());
    Settings.saveDouble("DriveTrain", "kvVoltsSecondsPerMeter", _drive_Train.getkvVoltSecondsPerMeter());
    Settings.saveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", _drive_Train.getkaVoltSecondsSquaredPerMeter());
  }
}
