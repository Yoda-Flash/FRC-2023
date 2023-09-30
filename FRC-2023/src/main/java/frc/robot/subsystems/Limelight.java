// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  // NetworkTableEntry ty = table.getEntry("ty");
  // NetworkTableEntry ta = table.getEntry("ta");
  // NetworkTableEntry tv = table.getEntry("tv");

  double validTarget;

  double x;
  double y;
  double area;
  boolean ledMode;
  double pipeline;
  boolean setPipeline;

  public Limelight() {
  
  }

  public double getPipeline(){
    return pipeline;
  }

  public double getTV(){
    return validTarget;
  }

  public double getTX(){
    return x;
  }

  public double getTY(){
    return y;
  }

  public double getTA(){
    return area;
  }

  public boolean setPipeline(double setPipe){
    setPipeline = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("pipeline").setNumber(setPipe);
    return setPipeline;
  }

  public boolean turnOn(){
    ledMode = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("ledMode").setNumber(3);
    return ledMode;
  }

  public boolean turnOff(){
    ledMode = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("ledMode").setNumber(1);
    return ledMode;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var table = NetworkTableInstance.getDefault().getTable("limelight-ahs");
    // System.out.println("DEBUG: " + table.getEntry("pipeline").setDouble(1));
    x = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("tx").getDouble(0);
    y = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("ty").getDouble(0);
    area = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("ta").getDouble(0);
    validTarget = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("tv").getDouble(0);
    pipeline = NetworkTableInstance.getDefault().getTable("limelight-ahs").getEntry("getpipe").getDouble(0);

    //For pipeline setting

  // x = tx.getDouble(0.0);
  // y = ty.getDouble(0.0);
  // area = ta.getDouble(0.0);


    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight A", area);
    SmartDashboard.putNumber("Limelight V", validTarget);
    
  }
}
