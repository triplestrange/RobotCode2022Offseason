// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Calibration extends SequentialCommandGroup {
  /** Creates a new Calibration. */
  public Calibration(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    TrajectoryConfig config = new TrajectoryConfig(4.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setStartVelocity(0)
            .setEndVelocity(0);

    Trajectory meterTrajectory = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(

        ),
            // direction robot moves
            new Pose2d(0, -1, new Rotation2d(0)), config);

    SwerveControllerCommand meter = new SwerveControllerCommand(meterTrajectory,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {
          return new Rotation2d(0);
        },

        swerve::setModuleStates,

        swerve

    );

    addCommands(meter);
  }
}
