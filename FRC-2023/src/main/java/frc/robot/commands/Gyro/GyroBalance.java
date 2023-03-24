// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Gyro;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drivetrain;

// public class GyroBalance extends CommandBase {

//   private static final class Config{
//     public static final double kSpeed = 0.3;
//     public static final double kDeadband = 0.2;
//   }

//   private Drivetrain m_drivetrain;

//   private Timer m_timer = new Timer();

//   private double m_initPosition;
  
//   /** Creates a new GyroBalance. */
//   public GyroBalance(Drivetrain drivetrain) {
//     m_drivetrain = drivetrain;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_timer.start();
//     m_initPosition = m_drivetrain.getGyroAngle();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (m_timer.hasElapsed(1.0)){
//     if (m_drivetrain.getGyroAngle() - m_initPosition >= Config.kDeadband){
//       //Drive towards game piece hub
//       m_drivetrain.getDrive().arcadeDrive(Config.kSpeed, 0);
//     }
//     else if (m_drivetrain.getGyroAngle() - m_initPosition <= -Config.kDeadband){
//       //Drive away from game piece hub
//       m_drivetrain.getDrive().arcadeDrive(-Config.kSpeed, 0);
//     }
//   }
// }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_timer.reset();
//     m_drivetrain.getDrive().arcadeDrive(0,0);
//     m_initPosition = 0;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_drivetrain.getGyroAngle() - m_initPosition <= Config.kDeadband && m_drivetrain.getGyroAngle() - m_initPosition >= -Config.kDeadband && m_timer.hasElapsed(7);
//     //return false;
//   }
// }
