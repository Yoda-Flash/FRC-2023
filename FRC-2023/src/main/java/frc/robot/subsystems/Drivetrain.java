
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Drivetrain extends SubsystemBase {
  
  /** Creates a new Drivetrain. */
  private static final class Config{
    public static final int kLeftPrimaryID = 1;
    public static final int kRightPrimaryID = 3;
    public static final int kLeftSecondaryID = 2;
    public static final int kRightSecondaryID = 4;
  }
  private WPI_TalonFX m_leftPrimary = new WPI_TalonFX(Config.kLeftPrimaryID);
  private WPI_TalonFX m_rightPrimary = new WPI_TalonFX(Config.kRightPrimaryID);
  private WPI_TalonFX m_leftSecondary = new WPI_TalonFX(Config.kLeftSecondaryID);
  private WPI_TalonFX m_rightSecondary = new WPI_TalonFX(Config.kRightSecondaryID);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);

  // private ADXRS450_Gyro m_gyro  = new ADXRS450_Gyro();
  private AHRS m_gyro  = new AHRS(SPI.Port.kMXP);


  public Drivetrain() {
    m_leftSecondary.follow(m_leftPrimary);
    m_rightSecondary.follow(m_rightPrimary);
    
    m_rightPrimary.setInverted(false);
    m_rightSecondary.setInverted(false);
    m_leftPrimary.setInverted(true);
    m_leftSecondary.setInverted(true);
  }


  public DifferentialDrive getDrive(){
    return m_drive;
  }
 
  public double getTicks(){
    return m_leftPrimary.getSelectedSensorPosition();
  }

  public void resetTicks(){
    m_leftPrimary.setSelectedSensorPosition(0.0);
  }




  public void calibrateGyro(){
    // essentially reset it to 0
    m_gyro.calibrate();
  }

  public double getTotalAngle(){
    // mod 360 if you want an "actual value"
    return m_gyro.getAngle();
  }

  // this is the method/angle used for the auto balance -roborio is tilted
  public double getAngle(){
    // returns angle from 0-360
    return getRoll();
    // return m_gyro.getAngle() - 360*(int)(m_gyro.getAngle()/360);
  }


  public double getPitch(){
    // higher pitch - bot is doing a wheelie
    // lower pitch - bot just landed a jump

    return m_gyro.getPitch(); 
  }

  public double getRoll(){
    // higher roll - Bot is leaning towards the right
    // lower roll - bot is leaning towards the left

    return m_gyro.getRoll();
  }

  public double getRegRoll(){
    // regular roll - essentially so ur roll isnt at 10000 degrees
    return m_gyro.getRoll() - 360*(int)(m_gyro.getAngle()/360);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
