// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive_Train;
import frc.robot.util.TrajectoryCache;

public class AutonDrivePath extends CommandBase {
  private Drive_Train _drivetrain;
  private RamseteCommand _ramsete;
  private String _path;

  public AutonDrivePath(Drive_Train drivetrain, String path) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    _path = path;

    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _ramsete = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_ramsete == null) {
      _ramsete = createRamseteCommand();
      _ramsete.initialize();
      return;
    }
    _ramsete.execute();
  }

  private RamseteCommand createRamseteCommand() {

    Trajectory trajectory = TrajectoryCache.get(_path);

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        _drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DrivetrainConstants.ksVolts,
            DrivetrainConstants.kvVoltSecondsPerMeter,
            DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        DrivetrainConstants.kDriveKinematics, _drivetrain::getWheelSpeeds,
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
        new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
        _drivetrain::tankDriveVolts, _drivetrain);

    _drivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _ramsete.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _ramsete.isFinished();
  }
}
