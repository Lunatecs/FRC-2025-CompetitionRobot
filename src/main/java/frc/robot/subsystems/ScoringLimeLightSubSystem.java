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

public class ScoringLimeLightSubSystem extends SubsystemBase {
  /** Creates a new ScoringLimeLightSubSystem. */
    private NetworkTable limelight;
  private NetworkTableEntry NetworkTx;
  private NetworkTableEntry NetworkTy;
  private NetworkTableEntry NetworkTa;
  
  public ScoringLimeLightSubSystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight-left");
    NetworkTx = limelight.getEntry("tx");
    NetworkTy = limelight.getEntry("ty");
    NetworkTa = limelight.getEntry("ta");
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("vision-left: x = ", this.GetTx() + ", y = " + this.GetTy());
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
