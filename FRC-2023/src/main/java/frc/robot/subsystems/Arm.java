// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static class Config{
    public static final int kArmMotorID = 6;
    public static final double kArmEncoderTopLimit = 60; //In Encoder ticks

    public static final double kS = 0.01;
    public static final double kV = 0.01;
    public static final double kG = 0.01;
    public static final double kA = 0.01;
  }
  
  private CANSparkMax m_armMotor = new CANSparkMax(Config.kArmMotorID, MotorType.kBrushless);
  private DigitalInput m_bottomSwitch = new DigitalInput(0);

  public Arm() {

   //m_armMotor.setInverted(true);
   SmartDashboard.putData("Arm/Cancel", new InstantCommand(() -> System.out.println("Canceling commands on Arm...."), this));

  }

  public boolean getLowerLimit(){
    return m_bottomSwitch.get();
  }

  public double getEncoderLimitUp() {
    return Config.kArmEncoderTopLimit;
  }

  public double getEncoderTicks(){
    return m_armMotor.getEncoder().getPosition();
  }

  public void resetEncoderTicks(){
    m_armMotor.getEncoder().setPosition(0);
  }

  public void setSpeed(double speed){
    if (getLowerLimit() && speed < 0) m_armMotor.set(0);
    else if (getEncoderTicks() >= Config.kArmEncoderTopLimit && speed > 0) m_armMotor.set(0);
    else m_armMotor.set(speed);
  }

  public void setSpeedDangerous(double speed) { //NEVER use this
    m_armMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Lower Limit", getLowerLimit());
    SmartDashboard.putNumber("Arm Encoder Ticks", getEncoderTicks());
    // This method will be called once per scheduler run
  }
}
