// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubSystem extends SubsystemBase {
  /** Creates a new ElevatorSubSystem. */
  private TalonFX motor1;
  private TalonFX motor2;

  private final DigitalInput elevatorLimitSwitch = new DigitalInput(Constants.ElevatorSubSystem.INPUT_ID);


  public ElevatorSubSystem() {
    motor1 = new TalonFX(Constants.ElevatorSubSystem.CAN_ID_MOTOR1);
    motor2 = new TalonFX(Constants.ElevatorSubSystem.CAN_ID_MOTOR2);
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);

    motor2.setControl(new Follower(motor1.getDeviceID(), true));
  }

  public void setSpeed(double speed){
    motor1.set(speed);
  }

  public boolean getLimitSwitch(){
    return elevatorLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
