// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.Field;
import frc.robot.field.ReefPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobotToReef_PID extends Command {
  /** Creates a new AlignRobotToReef_PID. */
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest.RobotCentric robotCentricDrive;

  Pose2d currentPose;
  Pose2d goalPose;
  Command run;
  String side;  

  PIDController pidStrafe;
  PIDController pidTranslate;
  PIDController pidRotation;

  double MaxSpeed = 3.0;
  double MaxAngularRate = 3.0;
  boolean isFinished;

  public AlignRobotToReef_PID(CommandSwerveDrivetrain swerve, SwerveRequest.RobotCentric robotCentricDrive, String side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.robotCentricDrive = robotCentricDrive;
    this.side = side;

    pidStrafe = new PIDController(.00625, 0, 0);
    pidStrafe.setSetpoint(0.0);
    pidStrafe.setTolerance(1);
    pidTranslate = new PIDController(.05, 0, 0);
    pidTranslate.setSetpoint(0.0);
    pidTranslate.setTolerance(1);
    pidRotation = new PIDController(.015, 0, 0);
    pidRotation.setSetpoint(0.0);
    pidRotation.setTolerance(1);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    currentPose = swerve.getState().Pose;
    ReefPose reefPose = Field.getField().getClosestReefPose(swerve.getState().Pose);
    if (side.equals("right")){
      goalPose =  reefPose.getRightPose();
    }
    if (side.equals("left")){
      goalPose =  reefPose.getLeftPose();
    }
    SmartDashboard.putString("PID Tracking goalPose", goalPose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe = pidStrafe.calculate(goalPose.getX());
    final double strafeSpeed = MathUtil.clamp(strafe, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("strafe", strafeSpeed);

    double translate = pidTranslate.calculate(goalPose.getY());
    final double translateSpeed = -MathUtil.clamp(translate, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("translate", translateSpeed);

    double rotation = pidRotation.calculate(goalPose.getRotation().getDegrees());
    final double rotationSpeed = MathUtil.clamp(rotation, -1.0, 1.0) * MaxAngularRate;
    SmartDashboard.putNumber("rotation", rotationSpeed);

    if(pidRotation.atSetpoint()){
      robotCentricDrive.withRotationalRate(0);
    } else {
      robotCentricDrive.withRotationalRate(rotationSpeed);
    }
    if(pidStrafe.atSetpoint()) {
      robotCentricDrive.withVelocityY(0);
    } else {
      robotCentricDrive.withVelocityY(strafeSpeed);
    }
    if(pidTranslate.atSetpoint()) {
      robotCentricDrive.withVelocityX(0);
    } else {
      robotCentricDrive.withVelocityX(translateSpeed);
    }

    if (pidRotation.atSetpoint() && pidStrafe.atSetpoint() && pidTranslate.atSetpoint()) {
      isFinished = true;
    }
    
    swerve.setControl(robotCentricDrive);

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
