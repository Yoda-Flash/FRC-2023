// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerIntake;

public class RunIntake extends CommandBase {
  private double startTime = 0;
  private RollerIntake m_rollerIntake;
  /** Creates a new RunIntake. */
  public RunIntake(RollerIntake rollerIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rollerIntake = rollerIntake;
    addRequirements(m_rollerIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollerIntake.setIntakeOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerIntake.setIntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > 1.75;
  }
}
