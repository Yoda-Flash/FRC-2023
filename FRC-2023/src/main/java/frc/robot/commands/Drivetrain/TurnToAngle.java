// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  private Drivetrain m_drivetrain;
  private double position;
  private double m_angle;
  private double m_deadband = 0.05;

  public TurnToAngle(Drivetrain drivetrain, double angle) {
    // angle is in degrees
    // turn clockwise

    // for consistency w other files -- use roll

    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_angle = angle;


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_drivetrain.getRegRoll();

    // somehow change roll
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // only rotational speed
    // distance from turning (assuming that we're rotating in the positive direction) -- TODO - test

    // basic PID (only P, no I or D)
    m_drivetrain.getDrive().arcadeDrive(0, 1.0*(m_angle - (m_drivetrain.getRegRoll() - position)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO - test this to make sure that the robot is spinning in the right direction
    // put some smart dashboard stuff on here
    return m_angle - (m_drivetrain.getRegRoll() - position) < m_deadband;
  }
}
