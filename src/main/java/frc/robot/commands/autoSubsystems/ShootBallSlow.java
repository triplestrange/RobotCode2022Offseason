// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootBallSlow extends CommandBase {
  private Shooter shooter;
  private Conveyor conveyor;

  /** Creates a new ShootBall. */
  public ShootBallSlow(Shooter shooter, Conveyor conveyor) {
    addRequirements(shooter, conveyor);
    this.shooter = shooter;
    this.conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean extended = SmartDashboard.getBoolean("HoodExtended", false);
    conveyor.runConveyor();

    shooter.setShooter(2000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
