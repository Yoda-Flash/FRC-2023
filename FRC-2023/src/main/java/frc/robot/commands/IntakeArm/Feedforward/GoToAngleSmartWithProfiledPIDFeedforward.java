// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArm.Feedforward;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GoToAngleSmartWithProfiledPIDFeedforward extends CommandBase {
  
  private static final class Config{
    public static final double kP = 0.0125;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.0125;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kMaxVelocity = 10;
    public static final double kMaxAccel = 10;
  }

  private Arm m_arm;
  private ProfiledPIDController m_pid = new ProfiledPIDController(Config.kP, Config.kI, Config.kD, new TrapezoidProfile.Constraints(0, 0));
  private ArmFeedforward m_feedforward = new ArmFeedforward(Config.kS, Config.kG, Config.kV, Config.kA);

  private double m_setpoint;
  private double m_encoderTicks;
  private double m_speed;
  private double m_default;

  /** Creates a new GoToAngleSmartWithProfiledPIDFeedforward. */
  public GoToAngleSmartWithProfiledPIDFeedforward() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
