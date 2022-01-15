// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HopperBack;
import frc.robot.commands.HopperGo;
import frc.robot.commands.HopperStop;
import frc.robot.commands.RobotDrive;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

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
  private final Drive_Train _drive_Train = new Drive_Train();
  private final Joystick m_driver = new Joystick(0);
  private final Joystick m_operator = new Joystick(1); 
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();
  private final Limelight m_limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    _drive_Train.setDefaultCommand(new RobotDrive(_drive_Train, m_driver));
    //m_intake.setDefaultCommand(new IntakeStop(m_intake)); 
    m_hopper.setDefaultCommand(new HopperStop(m_hopper));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //OPERATOR
    // new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(m_intake)); 
    // new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(m_intake)); 

    new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new HopperGo(m_hopper));
    new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(m_hopper));

   
    new JoystickButton(m_operator, Constants.JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> m_limelight.toggleBypass()));

    //DRIVER

    // new JoystickButton(m_driver, Constants.JoystickConstants.A).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Green)); 
    // new JoystickButton(m_driver, Constants.JoystickConstants.Y).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Yellow));
    // new JoystickButton(m_driver, Constants.JoystickConstants.X).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Blue));
    // new JoystickButton(m_driver, Constants.JoystickConstants.B).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Red));

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
