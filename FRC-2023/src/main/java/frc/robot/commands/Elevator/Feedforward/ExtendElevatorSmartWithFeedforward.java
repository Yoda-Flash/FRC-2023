// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Feedforward;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ExtendElevatorSmartWithFeedforward extends CommandBase {
  
  private static final class Config{
    public static final double kS = 0;
    public static final double kG = 0.2;
    public static final double kV = 0;
    public static final double kA = 0;
  }

  // private double kS;
  // private double kG;
  // private double kV;
  // private double kA;

  private Elevator m_elevator;
  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(Config.kS, Config.kG, Config.kV, Config.kA);

  private double m_setpoint;
  private double m_velocity = 500;
  private double m_accel = 500;
  private double m_speed;
  private double m_default;

  /** Creates a new ExtendElevatorSmartWithFeedforward. */
  public ExtendElevatorSmartWithFeedforward(Elevator elevator, double angle) {
    m_elevator = elevator;
    m_setpoint = angle;
    m_default = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   SmartDashboard.putNumber("Elevator/setpointTicks", m_default);
  //  SmartDashboard.putNumber("Elevator kS", kS);
  //  SmartDashboard.putNumber("Elevator kG", kG);
  //  SmartDashboard.putNumber("Elevator kV", kV);
  //  SmartDashboard.putNumber("Elevator kA", kA);
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
  //  kS = SmartDashboard.getNumber("Elevator kS", kS);
  //  kG = SmartDashboard.getNumber("Elevator kG", kG);
  //  kV = SmartDashboard.getNumber("Elevator kV", kV);
  //  kA = SmartDashboard.getNumber("Elevator kA", kA);
   m_setpoint = SmartDashboard.getNumber("Elevator/setpointTicks", m_default);
   m_speed = m_feedforward.calculate(m_velocity, m_accel);
   SmartDashboard.putNumber("Calculated Speed", m_speed);

   m_elevator.setMotor(m_speed);
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   m_elevator.setMotor(0);
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
  return m_elevator.getEncoderTicks() >= m_setpoint;
 }
}
