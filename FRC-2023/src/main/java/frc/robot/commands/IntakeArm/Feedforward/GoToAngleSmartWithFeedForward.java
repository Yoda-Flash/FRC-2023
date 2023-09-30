// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArm.Feedforward;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GoToAngleSmartWithFeedForward extends CommandBase {
  
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

  private Arm m_arm;
  private ArmFeedforward m_feedforward = new ArmFeedforward(Config.kS, Config.kG, Config.kV, Config.kA);

  private double m_setpoint;
  private double m_encoderTicks;
  private double m_speed;
  private double m_default;
  private double m_velocity = 500;
  private double m_accel = 500;

  /** Creates a new GoToAngleSmartWithFeedForward. */
  public GoToAngleSmartWithFeedForward(Arm arm, double angle) {
    m_arm = arm;
    m_setpoint = angle;
    m_default = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }
  
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Arm/setpointTicks", m_default);
    // SmartDashboard.putNumber("Arm kS", kS);
    // SmartDashboard.putNumber("Arm kG", kG);
    // SmartDashboard.putNumber("Arm kV", kV);
    // SmartDashboard.putNumber("Arm kA", kA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // kS = SmartDashboard.getNumber("Arm kS", kS);
    // kG = SmartDashboard.getNumber("Arm kG", kG);
    // kV = SmartDashboard.getNumber("Arm kV", kV);
    // kA = SmartDashboard.getNumber("Arm kA", kA);

    m_setpoint = SmartDashboard.getNumber("Arm/setpointTicks", m_default);
    m_encoderTicks = m_arm.getEncoderTicks();
    m_speed = m_feedforward.calculate(m_setpoint, m_velocity, m_accel);
    if (m_speed > .5) m_speed = 0.5;
    SmartDashboard.putNumber("Calculated Speed", m_speed);

    m_arm.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getEncoderTicks() >= m_setpoint;
  }
}
