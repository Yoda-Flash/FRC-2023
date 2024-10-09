// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AutoTrackPole extends CommandBase {
  private static final class Config{
    private static final double kP = 0.017; 
    private static final double kI = 0.01;
    private static final double kD = 0; 
  }

  private Limelight m_limelight;
  private Drivetrain m_drivetrain;
  private PIDController m_pidController = new PIDController(Config.kP, Config.kI, Config.kD);
  
  /** Creates a new AutoTrackPole. */
  public AutoTrackPole(Limelight limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drivetrain = drivetrain; 
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_limelight.getTV() == 1.0) {
      //if(m_limelight.getTX() <=)
      m_drivetrain.getDrive().arcadeDrive(0,-m_pidController.calculate(m_limelight.getTX(), 0));
  

    //   if (m_limelight.getTX() >= 1.5){
    //     //If target is to the right, move to the left
    //     m_drivetrain.getDrive().arcadeDrive(0, 0.3);
    //   }
    //   if (m_limelight.getTX() <= -1.5){

    //     m_drivetrain.getDrive().arcadeDrive(0, -0.3);
    //   }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_limelight.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.getTX() <= 1.5 && m_limelight.getTX() >= -1.5 && m_limelight.getTV() == 1;
  }
}
