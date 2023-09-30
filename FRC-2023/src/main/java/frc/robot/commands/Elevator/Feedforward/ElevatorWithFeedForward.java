// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Feedforward;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorWithFeedForward extends CommandBase {
  //For full extension

  private static final class Config{
    public static final double kS = 0.1;
    public static final double kG = 0.1;
    public static final double kV = 0.1;
    public static final double kA = 0.1;
  }

  private Elevator m_elevator;
  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Config.kS, Config.kG, Config.kV, Config.kA);

  private double m_setpoint = 300;
  private double m_velocity = 5;
  private double m_accel = 5;

  /** Creates a new ElevatorWithFeedForward. */
  public ElevatorWithFeedForward(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setMotor(m_feedforward.calculate(m_velocity, m_accel));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getEncoderTicks() >= m_elevator.getEncoderLimitUp();
  }
}