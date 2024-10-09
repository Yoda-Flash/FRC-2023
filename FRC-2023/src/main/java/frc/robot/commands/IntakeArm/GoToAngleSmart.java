// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;

public class GoToAngleSmart extends CommandBase {
  /** Creates a new GoToAngle. */

  private static final class Config{
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int kDownButtonID = 7;
  }

  private Arm m_arm;
  private PIDController m_controller = new PIDController(Config.kP, Config.kI, Config.kD);

  private Joystick m_joystick;
  private JoystickButton m_downButton;

  private double m_setpoint;
  private double m_encoderTicks;
  private double m_speed;
  private double m_default;
  
  public GoToAngleSmart(Arm arm, double angle, Joystick joystick) {
      
    m_arm = arm;
    m_setpoint = angle; //NOTE, this is in encoder ticks (ideally a fraction of max encoder ticks)
    m_default = angle;

    m_joystick = joystick;
    m_downButton = new JoystickButton(m_joystick, Config.kDownButtonID);
    
    addRequirements(m_arm);
  }

  public GoToAngleSmart(Arm arm, double angle) {
      
    m_arm = arm;
    m_setpoint = angle; //NOTE, this is in encoder ticks (ideally a fraction of max encoder ticks)
    m_default = angle;

    // NOTE THIS IS A FAKE JOYSTICK AND SHOULD NEVER BE USED BECAUSE BAD. BUT DONT REMOVE IT OR CRASH LMAO
    m_joystick = new Joystick(4);
    m_downButton = new JoystickButton(m_joystick, 0);
    
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Arm/setpointTicks", m_default);
    m_setpoint = SmartDashboard.getNumber("Arm/setpointTicks", m_default);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_downButton.getAsBoolean()) m_setpoint -= 0.075;
    SmartDashboard.putNumber("Arm/setpointTicks", m_setpoint);

    SmartDashboard.putData("PID/Arm PID", m_controller);

    // m_setpoint = SmartDashboard.getNumber("Arm/setpointTicks", m_default); NOTE this overrides 
    m_encoderTicks = m_arm.getEncoderTicks();
    m_speed = m_controller.calculate(m_encoderTicks, m_setpoint);
    if (Math.abs(m_speed) > 0.6) m_speed = Math.signum(m_speed)*0.6;
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
    return false;
  }
}
