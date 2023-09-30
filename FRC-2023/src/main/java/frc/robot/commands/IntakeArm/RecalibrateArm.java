// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RecalibrateArm extends CommandBase {

  private Arm m_arm;
  
  /** Creates a new RecalibrateArm. */
  public RecalibrateArm(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!m_arm.getLowerLimit()) m_arm.setSpeed(-0.15);
    else m_arm.setSpeed(0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0);
    m_arm.resetEncoderTicks();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getLowerLimit();
  }
}