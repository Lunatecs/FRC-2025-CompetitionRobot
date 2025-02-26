// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ScoringLimeLightSubSystemRight extends SubsystemBase {
  /** Creates a new ScoringLimeLightSubSystemRight. */
  private NetworkTable limelight;
  private NetworkTableEntry NetworkTx;
  private NetworkTableEntry NetworkTy;
  private NetworkTableEntry NetworkTa;
  private double[] botpose = new double[6];
  private String name = "limelight-right";

  public ScoringLimeLightSubSystemRight() {
    limelight = NetworkTableInstance.getDefault().getTable(name);
    NetworkTx = limelight.getEntry("tx");
    NetworkTy = limelight.getEntry("ty");
    NetworkTa = limelight.getEntry("ta");
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    botpose = LimelightHelpers.getBotPose_TargetSpace(name);
    SmartDashboard.putString("bot pose target RIGHT", botpose[0] + " " + botpose[2]+ " " +  botpose[4]);
    SmartDashboard.putNumber("limelight RIGHT x", getTranslationX());
    SmartDashboard.putNumber("limelight RIGHT y", getTranslationY());
  }

  public double getTranslationX() {
    return botpose[0];
  }

  public double getTranslationY() {
    return botpose[2];
  }


  public double getYaw() {
    return botpose[4];
  }

  public double GetTx(){
    return NetworkTx.getDouble(0.0);
    }
  
  public double GetTy(){
    return NetworkTy.getDouble(0.0);
  }
  
  public double GetTa(){
    return NetworkTa.getDouble(0.0);
  }
}
