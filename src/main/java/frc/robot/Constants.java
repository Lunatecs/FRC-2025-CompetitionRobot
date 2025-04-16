// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static double DEFAULT_ALGEA_INTAKE = .1;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CoralGroundIntakeSubSystemConstants{
    public static final int CAN_ID_INTAKE = 13;
  }

  public static class CarriageSubSystemConstants {
    public static final int CAN_ID_CARRIAGE =11;
  }

  public final class CoralGroundIntakePivotSubSystemConstants{
    public static final int CAN_ID_INTAKE_PIVOT = 12;
  }

  public static class CoralOutakeSubSystemConstants{
    public static final int CAN_ID_OUTTAKE = 15;
    public static final int CAN_ID_SENSOR = 22;
  }

  public static class CoralAlignmentSubSystemConstants{
    public static final int CAN_ID_SENSOR2 = 21;
  }

  public static class ClimberSubSystemConstants{
    public static final int CAN_ID_CLIMBER = 20;
    //switched from 17 to 20
  }

  public static class ElevatorSubSystemConstants{
    public static final int CAN_ID_MOTOR1 = 9;
    public static final int CAN_ID_MOTOR2 = 10;

    public static final int INPUT_ID = 0;
  } 

  public static class CoralFeederSubSystemConstants{
    public static final int CAN_ID_FEEDER = 14;
  }

  //public static class CoralHopperSubSystemConstants{
    //public static final int CAN_ID_HOPPER = 20;
    //Switched from 20 to 17, because the hopper motor is the climber
 // }

  public static class AlgaePivotSubSystemConstants{
    public static final int CAN_ID_ALGAE_PIVOT = 19;
  }

  public static class AlgaeLiberatorSubSystemConstants{
    public static final int CAN_ID_ALGAE_LIBERATOR = 16;
  }

  public static class LEDSubSystemConstants{
    public static final int PWM_ID_LED = 9;
  }
}
