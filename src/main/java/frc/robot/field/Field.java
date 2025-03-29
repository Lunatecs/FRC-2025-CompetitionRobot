// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.field;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Field {

    private static HashMap<Integer,ReefPose> blueNear = new HashMap<>();
    private static HashMap<Integer, ReefPose> redNear = new HashMap<>();
    private static HashMap<Integer, ReefPose> blueFar = new HashMap<>();
    private static HashMap<Integer, ReefPose> redFar = new HashMap<>();

    private static Field  field = new Field();


    private Field(){
        blueNear.put(17, new ReefPose(17, 
            new Pose2d(3.994, 2.83, Rotation2d.fromDegrees(60)), 
            new Pose2d(3.709, 3.007, Rotation2d.fromDegrees(60)), 
            3.85, 2.923));

        blueNear.put(18, new ReefPose(18,
         new Pose2d(3.2, 3.865, Rotation2d.fromDegrees(0)),
          new Pose2d(3.2, 4.193, Rotation2d.fromDegrees(0)),
          3.227, 4.025));

        blueNear.put(19, new ReefPose(19,
         new Pose2d(3.71, 5.044, Rotation2d.fromDegrees(-60)),
          new Pose2d(3.994, 5.21, Rotation2d.fromDegrees(-60)),
           3.846, 5.12));

        blueNear.put(20, new ReefPose(20,
         new Pose2d(4.984, 5.209, Rotation2d.fromDegrees(-120)),
          new Pose2d(5.266, 5.044, Rotation2d.fromDegrees(-120)),
           5.128, 5.122));

        blueNear.put(21, new ReefPose(21,
         new Pose2d(5.763, 4.192, Rotation2d.fromDegrees(180)),
          new Pose2d(5.763, 3.863, Rotation2d.fromDegrees(180)),
           5.757, 4.017));

        blueNear.put(22, new ReefPose(22,
         new Pose2d(5.264, 3.004, Rotation2d.fromDegrees(120)),
         new Pose2d(4.981, 2.841, Rotation2d.fromDegrees(120)),
          5.125, 2.924));

        redNear.put(6, new ReefPose(6,
         new Pose2d(13.842, 3.005, Rotation2d.fromDegrees(120)),
          new Pose2d(13.558, 2.84, Rotation2d.fromDegrees(120)),
           13.703, 2.925));

        redNear.put(7, new ReefPose(7,
         new Pose2d(14.38, 4.192, Rotation2d.fromDegrees(180)),
          new Pose2d(14.38, 3.865, Rotation2d.fromDegrees(180)),
           14.361, 4.04));

        redNear.put(8, new ReefPose(8,
         new Pose2d(13.579, 5.248, Rotation2d.fromDegrees(-120)),
          new Pose2d(13.863, 5.083, Rotation2d.fromDegrees(-120)),
           13.72, 5.167));

        redNear.put(9, new ReefPose(9,
         new Pose2d(12.261, 5.083, Rotation2d.fromDegrees(-60)),
          new Pose2d(12.546, 5.251, Rotation2d.fromDegrees(-60)),
           12.402, 5.166));

        redNear.put(10, new ReefPose(10,
        new Pose2d(11.749, 3.865, Rotation2d.fromDegrees(0)),
        new Pose2d(11.749, 4.192, Rotation2d.fromDegrees(0)),
           11.749, 4.015));

        redNear.put(11, new ReefPose(11,
         new Pose2d(12.571, 2.841, Rotation2d.fromDegrees(60)),
        new Pose2d(12.284, 3.007, Rotation2d.fromDegrees(60)),
           12.433, 2.918));


        blueFar.put(17, new ReefPose(17, 
           new Pose2d(3.964, 2.789, Rotation2d.fromDegrees(60)), 
           new Pose2d(2.789, 3.682, Rotation2d.fromDegrees(60)), 
           3.85, 2.923));

       blueFar.put(18, new ReefPose(18,
        new Pose2d(3.175, 3.865, Rotation2d.fromDegrees(0)),
         new Pose2d(3.175, 4.193, Rotation2d.fromDegrees(0)),
         3.227, 4.025));

       blueFar.put(19, new ReefPose(19,
        new Pose2d(3.687, 5.081, Rotation2d.fromDegrees(-60)),
         new Pose2d(3.969, 5.258, Rotation2d.fromDegrees(-60)),
          3.846, 5.12));

       blueFar.put(20, new ReefPose(20,
        new Pose2d(5.01, 5.26, Rotation2d.fromDegrees(-120)),
         new Pose2d(5.293, 5.089, Rotation2d.fromDegrees(-120)),
          5.128, 5.122));

       blueFar.put(21, new ReefPose(21,
        new Pose2d(5.814, 4.912, Rotation2d.fromDegrees(180)),
         new Pose2d(5.809, 3.858, Rotation2d.fromDegrees(180)),
          5.757, 4.017));

       blueFar.put(22, new ReefPose(22,
        new Pose2d(5.295, 2.957, Rotation2d.fromDegrees(120)),
        new Pose2d(5.005, 2.802, Rotation2d.fromDegrees(120)),
         5.125, 2.924));

       redFar.put(6, new ReefPose(6,
        new Pose2d(13.862, 2.97, Rotation2d.fromDegrees(120)),
         new Pose2d(13.578, 2.805, Rotation2d.fromDegrees(120)),
          13.703, 2.925));

       redFar.put(7, new ReefPose(7,
        new Pose2d(14.405, 4.192, Rotation2d.fromDegrees(180)),
         new Pose2d(14.405, 3.865, Rotation2d.fromDegrees(180)),
          14.361, 4.04));

       redFar.put(8, new ReefPose(8,
        new Pose2d(13.59, 5.262, Rotation2d.fromDegrees(-120)),
         new Pose2d(13.877, 5.107, Rotation2d.fromDegrees(-120)),
          13.72, 5.167));

       redFar.put(9, new ReefPose(9,
        new Pose2d(12.249, 5.107, Rotation2d.fromDegrees(-60)),
         new Pose2d(12.534, 5.272, Rotation2d.fromDegrees(-60)),
          12.402, 5.166));

       redFar.put(10, new ReefPose(10,
       new Pose2d(11.724, 3.865, Rotation2d.fromDegrees(0)),
       new Pose2d(11.724, 4.192, Rotation2d.fromDegrees(0)),
          11.749, 4.015));

       redFar.put(11, new ReefPose(11,
        new Pose2d(12.532, 2.786, Rotation2d.fromDegrees(60)),
       new Pose2d(12.255, 2.958, Rotation2d.fromDegrees(60)),
          12.433, 2.918));
    }

    public static Field getField() {
        return field;
    }

    public ReefPose getClosestReefPose(Pose2d robotPose) {
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Blue);
        HashMap<Integer,ReefPose> reefMap = null;
        if(color.equals(Alliance.Blue)) {
            reefMap = Field.blueNear;
        } else {
            reefMap = Field.blueNear; //Field.redNear;
        }

        Set<Entry<Integer,ReefPose>> values = reefMap.entrySet();

        Iterator<Entry<Integer,ReefPose>> iter = values.iterator();

        double shortest = 1000;
        int aprilTagNum = -1;
        String lengths = "";
        while(iter.hasNext()) {
            Entry<Integer,ReefPose> value = iter.next();
            double length = getLineLength(value.getValue().getCenterX(), robotPose.getX(), 
                                            value.getValue().getCenterY(), robotPose.getY());
            lengths = value.getValue().getAprilTagNumber() + ": " + length + "--";
            if(length < shortest){
                shortest = length;
                aprilTagNum = value.getValue().getAprilTagNumber();
            }

        }

        SmartDashboard.putString("lengths", lengths);

        return reefMap.get(aprilTagNum);
    }

    public double getLineLength(double x1, double x2, double y1, double y2){
        double lineLength = Math.sqrt(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2));
        return lineLength;
    }

}
