// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorLevelThreeCommand;
import frc.robot.commands.ElevatorLevelTwoCommand;
import frc.robot.commands.FullAlignLeftLimeLight;
import frc.robot.commands.FullAlignRightLimeLight;
import frc.robot.commands.GetCoralSubstationCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.IntakePivotAlgaeCommand;
import frc.robot.commands.ManualClimbCommand;
import frc.robot.commands.RaiseIntakeCommand;
import frc.robot.commands.ReefTrackingCommand;
import frc.robot.commands.testAuto3Piece;
import frc.robot.commands.AlignRobotToTagLeftLimeLight;
import frc.robot.commands.AlignRobotToTagRightLimeLight;
import frc.robot.commands.AutoDeliverCommand;
import frc.robot.commands.AutoTargetScoreLeftPoleL4Sequence;
import frc.robot.commands.AutoTargetScoreRightPoleL4Sequence;
import frc.robot.commands.AutoTrackToReef;
import frc.robot.commands.DropIntakeCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorLevelFourCommand;
import frc.robot.commands.ElevatorLevelOneCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralFeederSubSystem;
import frc.robot.subsystems.CoralGroundIntakePivotSubSystem;
import frc.robot.subsystems.CoralGroundIntakeSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.ScoringLimeLightSubSystemLeft;
import frc.robot.subsystems.ScoringLimeLightSubSystemRight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.SwerveDriveBrake xWheels = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private final CommandPS5Controller operator = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private CoralGroundIntakeSubSystem coralIntake = new CoralGroundIntakeSubSystem();
    private CoralFeederSubSystem coralFeeder = new CoralFeederSubSystem(); 
    private CarriageSubSystem coralCarriage = new CarriageSubSystem();
    private CoralOutakeSubSystem coralOutake = new CoralOutakeSubSystem();
    private final ElevatorSubSystem elevator = new ElevatorSubSystem();
    private final CoralGroundIntakePivotSubSystem pivot = new CoralGroundIntakePivotSubSystem();
    private final ScoringLimeLightSubSystemLeft limelightLeft = new ScoringLimeLightSubSystemLeft();
    private final ScoringLimeLightSubSystemRight limelightRight = new ScoringLimeLightSubSystemRight();
    //private final ClimberSubSystem climber = new ClimberSubSystem();
    //PathPlannerPath path = PathPlannerPath.fromPathFile("LEFTPATH");


    public RobotContainer() {
        
        NamedCommands.registerCommand("ScoreL4", new AutoDeliverCommand(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 70.65)); //70.5
        NamedCommands.registerCommand("Station Pick Up Command", new GetCoralSubstationCommand(elevator, coralOutake, coralCarriage));
        NamedCommands.registerCommand("Target and Score LEFT Pole", new AutoTargetScoreLeftPoleL4Sequence(limelightRight, drivetrain, elevator, coralOutake, robotCentricDrive, MaxSpeed, MaxAngularRate));
        NamedCommands.registerCommand("Target and Score RIGHT Pole", new AutoTargetScoreRightPoleL4Sequence(limelightLeft, drivetrain, elevator, coralOutake, robotCentricDrive, MaxSpeed, MaxAngularRate));

        NamedCommands.registerCommand("MoveIntakeOut", new RaiseIntakeCommand(pivot));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        //NamedCommands.registerCommand("testpathing", new testAuto3Piece(path, limelightLeft, drivetrain, elevator, coralOutake, robotCentricDrive, MaxSpeed, MaxAngularRate));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.R3().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(driver.getLeftY() * MaxSpeed * 0.25)
                                                                    .withVelocityY(driver.getLeftX() * MaxSpeed * 0.25)
                                                                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));
        
        //driver.cross().whileTrue(new ReefTrackingCommand(drivetrain));
        driver.triangle().whileTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.25)
                                                                                .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.25)
                                                                               .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));

        //driver.square().whileTrue(new AutoTrackToReef(drivetrain));
        //climber.setDefaultCommand(new ManualClimbCommand(climber, () -> {
           //return operator.getLeftY();    
        //}));
       /*  driver.L1().onTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(driver.getLeftY() * MaxSpeed)
                                                                            .withVelocityY(driver.getLeftX() * MaxSpeed)
                                                                            .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
*/
        driver.R1().whileTrue(new FullAlignLeftLimeLight(limelightLeft, drivetrain, robotCentricDrive, MaxSpeed, MaxAngularRate));
        //driver.R1().whileTrue(new AutoTargetScoreRightPoleL4Sequence(limelightLeft, drivetrain, elevator, coralOutake, robotCentricDrive, MaxSpeed, MaxAngularRate));
        driver.L1().whileTrue(new FullAlignRightLimeLight(limelightRight, drivetrain, robotCentricDrive, MaxSpeed, MaxAngularRate));
        


        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        driver.triangle().and(driver.R1()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.triangle().and(driver.L1()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.square().and(driver.R1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.square().and(driver.L1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
        driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        //driver.R2().onTrue(new InstantCommand(()->{ coralIntake.setSpeed(1); coralFeeder.setSpeed(1); coralCarriage.setSpeed(1); coralOutake.setSpeed(1);},coralIntake,coralFeeder,coralCarriage,coralOutake))
        //.onFalse(new InstantCommand(()->{coralIntake.setSpeed(0); coralFeeder.setSpeed(0); coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralIntake,coralFeeder,coralCarriage,coralOutake));

        driver.R2().onTrue(new IntakeCoralCommand(pivot, coralIntake, coralFeeder, coralCarriage, coralOutake));


        driver.L2().onTrue(new InstantCommand(()->{ coralIntake.setSpeed(-1); coralFeeder.setSpeed(-1); coralCarriage.setSpeed(-1); coralOutake.setSpeed(-1);},coralIntake,coralFeeder,coralCarriage,coralOutake))
        .onFalse(new InstantCommand(()->{coralIntake.setSpeed(0); coralFeeder.setSpeed(0); coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralIntake,coralFeeder,coralCarriage,coralOutake));

        operator.L1().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(1); coralOutake.setSpeed(1);}, coralCarriage,coralOutake))
                    .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));
        
        operator.L2().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(0.3); coralOutake.setSpeed(0.3);}, coralCarriage,coralOutake))
        .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        operator.R2().onTrue(new InstantCommand(()->{elevator.setSpeed(-0.1);},elevator))
        .onFalse(new InstantCommand(()->{elevator.setSpeed(0);},elevator));

        operator.R1().onTrue(new InstantCommand(()->{elevator.setSpeed(0.3);},elevator))
        .onFalse(new InstantCommand(()->{elevator.setSpeed(0);},elevator));

        operator.triangle().onTrue(new ElevatorLevelFourCommand(elevator));
        operator.square().onTrue(new ElevatorLevelThreeCommand(elevator));
        operator.circle().onTrue(new ElevatorLevelTwoCommand(elevator));
        operator.cross().onTrue(new ElevatorLevelOneCommand(elevator));

        //operator.povUp().onTrue(new AutoDeliverCommand(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 71.5));
        operator.povUp().onTrue(new AutoDeliverCommand(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 70.0));
        operator.povDown().onTrue(new ElevatorDownCommand(elevator));
        driver.povUp().onTrue(new RaiseIntakeCommand(pivot));
        driver.povDown().onTrue(new DropIntakeCommand(pivot));

        operator.povRight().onTrue(new GetCoralSubstationCommand(elevator, coralOutake, coralCarriage));

        driver.povRight().onTrue(new IntakePivotAlgaeCommand(pivot));
        driver.povLeft().onTrue(new InstantCommand(()->{coralIntake.setSpeed(.3);}))
        .onFalse(new InstantCommand(()->{coralIntake.setSpeed(0);}));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
