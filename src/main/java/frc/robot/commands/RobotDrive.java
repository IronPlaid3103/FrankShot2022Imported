// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class RobotDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private Drive_Train _driveTrain;
  private Joystick _joystick;

  public RobotDrive(Drive_Train driveTrain, Joystick joystick) {
    _driveTrain = driveTrain;
    _joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.teleopDrive(_joystick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
