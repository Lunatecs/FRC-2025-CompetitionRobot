// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ScoringLimeLightSubSystemRight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobotToTagRightLimeLight extends Command {
  /** Creates a new AlignRobotToTag. */
  PIDController pidStrafe;
  PIDController pidTranslate;
  PIDController pidRotation;
  ScoringLimeLightSubSystemRight limelight;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  double MaxSpeed;
  double MaxAngularRate;
  boolean isFinished;

  public AlignRobotToTagRightLimeLight(ScoringLimeLightSubSystemRight limelight, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric, double MaxSpeed, double MaxAngularRate) {
    // Use addRequirements() here to declare subsystem dependencies.
    pidStrafe = new PIDController(.4, 0, 0); // Horizontal PID (NEEDS TO BE TUNED BETTER)
    pidStrafe.setSetpoint(0.07);
    pidStrafe.setTolerance(0.02);
    pidTranslate = new PIDController(.5, 0, 0); // Forward/Backward PID (NEEDS TO BE TUNED BETTER)
    pidTranslate.setSetpoint(-0.40);
    pidTranslate.setTolerance(0.1);
    pidRotation = new PIDController(.018, 0, 0); // Rotation PID
    pidRotation.setSetpoint(0);
    pidRotation.setTolerance(0.0);
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.drive = robotCentric;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe = pidStrafe.calculate(limelight.getTranslationX());
    final double strafeSpeed = MathUtil.clamp(strafe, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("strafe", strafeSpeed);

    double translate = pidTranslate.calculate(limelight.getTranslationY());
    final double translateSpeed = -MathUtil.clamp(translate, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("translate", translateSpeed);

    double rotation = pidRotation.calculate(limelight.getYaw());
    final double rotationSpeed = MathUtil.clamp(rotation, -1.0, 1.0) * MaxAngularRate;
    SmartDashboard.putNumber("rotation", rotationSpeed);

    SmartDashboard.putBoolean("LEFT POLE TRANSLATION AT SETPOINT", pidTranslate.atSetpoint());
    SmartDashboard.putBoolean("LEFT POLE STRAFE AT SETPOINT", pidStrafe.atSetpoint());

    if(pidRotation.atSetpoint()){
      drive.withRotationalRate(0);
    } else {
      drive.withRotationalRate(-rotationSpeed);
    }
    if(pidStrafe.atSetpoint()) {
      drive.withVelocityY(0);
    } else {
      drive.withVelocityY(-strafeSpeed);
    }
    if(pidTranslate.atSetpoint()) {
      drive.withVelocityX(0);
    } else {
      drive.withVelocityX(-translateSpeed);
      //drive.withVelocityX(0);
    }

    drivetrain.setControl(drive);

    if (pidTranslate.atSetpoint() && pidStrafe.atSetpoint()) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
