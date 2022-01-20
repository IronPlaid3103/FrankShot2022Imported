// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterV2.COLOR;
import frc.robot.util.Limelight;
import frc.robot.util.TrajectoryCache;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();
  private final Drive_Train _drive_Train = new Drive_Train(_gyro);
  private final Joystick _driver = new Joystick(0);
  private final Hopper _hopper = new Hopper();
  private final Intake _intake = new Intake();
  private final Limelight _limelight = new Limelight();
  private final ShooterV2 _shooter = new ShooterV2();

  private SendableChooser<String> m_ChallengeChooser = new SendableChooser<String>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    _drive_Train.setDefaultCommand(new RobotDrive(_drive_Train, _driver));
    _intake.setDefaultCommand(new IntakeStop(_intake)); 
    _hopper.setDefaultCommand(new HopperStop(_hopper));

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
    // // new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(m_intake)); 
    // // new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(m_intake)); 

    // new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new HopperGo(m_hopper));
    // new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(m_hopper));

   
    // new JoystickButton(m_operator, Constants.JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> m_limelight.toggleBypass()));

    //DRIVER
    new JoystickButton(_driver, Constants.JoystickConstants.LOGO_LEFT).whenPressed(new InstantCommand(() -> _gyro.reset()));

    //the buttons below are generally for testing purposes only
    new JoystickButton(_driver, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(_intake)); 
    new JoystickButton(_driver, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(_intake)); 

    new JoystickButton(_driver, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new HopperIdle(_hopper));
    new JoystickButton(_driver, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(_hopper));

    new JoystickButton(_driver, Constants.JoystickConstants.A).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Green)); 
    new JoystickButton(_driver, Constants.JoystickConstants.Y).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Yellow));
    new JoystickButton(_driver, Constants.JoystickConstants.X).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Blue));
    new JoystickButton(_driver, Constants.JoystickConstants.B).whileHeld(new ShooterV2Go(_shooter, _hopper, COLOR.Red));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
