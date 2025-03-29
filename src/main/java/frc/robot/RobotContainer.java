// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorLevelThreeCommand;
import frc.robot.commands.ElevatorLevelTwoAlgaeCommand;
import frc.robot.commands.ElevatorLevelTwoCommand;
import frc.robot.commands.GetCoralCommand;
import frc.robot.commands.NewAlageDown;
import frc.robot.commands.AlgaeFromGroundPivotCommand;
import frc.robot.commands.AlgaeFromReefPivotCommand;
import frc.robot.commands.AlgaePivotResetCommand;
import frc.robot.commands.AlignToReefPoseCommand;
import frc.robot.commands.BensalemAlgaeClutch;
import frc.robot.commands.BlinkAlignCommand;
import frc.robot.commands.CoralFromGroundPivotCommand;
import frc.robot.commands.CoralLevelOnePivotCommand;
import frc.robot.commands.DeliverCoralAtHeight;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorLevelFourCommand;
import frc.robot.commands.ElevatorLevelOneCommand;
import frc.robot.commands.ElevatorLevelThreeAlgaeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralAlignmentSubSystem;
import frc.robot.subsystems.CoralHopperSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.WhaleTailReleaseSubSystem;

public class RobotContainer {

    // Swerve Set Up
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // private final SwerveRequest.SwerveDriveBrake xWheels = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);

    // Subsystem Instantiation
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CarriageSubSystem coralCarriage = new CarriageSubSystem();
    private final CoralOutakeSubSystem coralOutake = new CoralOutakeSubSystem();
    private final ElevatorSubSystem elevator = new ElevatorSubSystem();
    private final CoralHopperSubSystem hopper = new CoralHopperSubSystem();
    private final AlgaePivotSubSystem pivot = new AlgaePivotSubSystem();
    private final AlgaeLiberatorSubSystem liberator = new AlgaeLiberatorSubSystem();
    private final CoralAlignmentSubSystem align = new CoralAlignmentSubSystem();
    private final WhaleTailReleaseSubSystem dropServo = new WhaleTailReleaseSubSystem();
    private final LEDSubSystem blink = new LEDSubSystem();

    public RobotContainer() {
        
        // Event Marker Commands for Path Planner
        NamedCommands.registerCommand("Station Pick Up Command", new GetCoralCommand(hopper, coralCarriage, coralOutake, elevator));
        NamedCommands.registerCommand("L1", new ElevatorLevelOneCommand(elevator));
        NamedCommands.registerCommand("L2", new ElevatorLevelTwoCommand(elevator));
        NamedCommands.registerCommand("L4", new ElevatorLevelFourCommand(elevator));
        NamedCommands.registerCommand("Elevator Down", new ElevatorDownCommand(elevator));
        NamedCommands.registerCommand("Score At L4", new DeliverCoralAtHeight(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 69.8)); //70.65

        // Auto Mode Set Up
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    public void dropWhaleTale() {
        dropServo.dropTail(true);
    }

    private void configureBindings() {
        // Default Commands
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed)
                    .withVelocityY(driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

        blink.setDefaultCommand(new BlinkAlignCommand(blink, align));

        drivetrain.registerTelemetry(logger::telemeterize);


        // DRIVER CONTROLS

        driver.cross().whileTrue(new AlignToReefPoseCommand(drivetrain));

        driver.triangle().onTrue(new InstantCommand(()->{this.dropServo.dropTail(true);}));
        //Slow Mode
        driver.R1().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(driver.getLeftY() * MaxSpeed * 0.25)
                                                                    .withVelocityY(driver.getLeftX() * MaxSpeed * 0.25)
                                                                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));
        // Robot Centric (at Slow Mode Speed)
        driver.L1().whileTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.15)
                                                                                .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.15)
                                                                               .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));

        // Field Centric Heading Reset
        driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.L2().whileTrue(new RunCommand(()-> {liberator.setSpeed(1); coralOutake.setSpeed(1);}, liberator, coralOutake))
                    .onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));

        driver.povDown().onTrue(new AlgaeFromGroundPivotCommand(pivot));

        driver.povUp().onTrue(new AlgaePivotResetCommand(pivot));

        driver.povLeft().onTrue(new CoralLevelOnePivotCommand(pivot));

        driver.povRight().onTrue(new CoralFromGroundPivotCommand(pivot));

        // OPERATOR CONTROLS
        // Coral Elevator Bindings
        operator.cross().onTrue(new ElevatorLevelOneCommand(elevator));
        operator.square().onTrue(new ElevatorLevelTwoCommand(elevator));
        operator.circle().onTrue(new ElevatorLevelThreeCommand(elevator));
        operator.triangle().onTrue(new ElevatorLevelFourCommand(elevator));
        
        // Elevator Down
        operator.povDown().onTrue(new ElevatorDownCommand(elevator));

        // Run End Effector
        operator.L1().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(1); coralOutake.setSpeed(1);}, coralCarriage,coralOutake))
                    .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        operator.L2().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(0.3); coralOutake.setSpeed(0.3);}, coralCarriage,coralOutake))
                        .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        // Reverse End Effector
        //operator.R2().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(-1); coralOutake.setSpeed(-1);}, coralCarriage,coralOutake))
                        //.onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        // Hopper
        operator.povRight().onTrue(new GetCoralCommand(hopper, coralCarriage, coralOutake, elevator));

        // Algae Bindings
        // ALGAE Elevator Commands
        operator.povLeft().onTrue(new BensalemAlgaeClutch(new ElevatorLevelTwoAlgaeCommand(elevator), pivot, liberator, coralOutake, elevator))
                                                .onFalse(new NewAlageDown(pivot, elevator, liberator, coralOutake));

        operator.povUp().onTrue(new BensalemAlgaeClutch(new ElevatorLevelThreeAlgaeCommand(elevator), pivot, liberator, coralOutake, elevator))
                                            .onFalse(new NewAlageDown(pivot, elevator, liberator, coralOutake));


        //operator.triangle().and(operator.R1()).onTrue(new ElevatorLevelFourCommand(elevator));

        //Algae Outtake Bindings
        operator.R1().whileTrue(new RunCommand(()-> {liberator.setSpeed(-0.4); coralOutake.setSpeed(-0.4);}, liberator, coralOutake))
            .onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));
        operator.R2().whileTrue(new RunCommand(()-> {liberator.setSpeed(1); coralOutake.setSpeed(1);}, liberator, coralOutake))
            .onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));
        
        //Algae Intake Bindings
        //operator.L2().and(operator.R1()).whileTrue(new RunCommand(()-> {liberator.setSpeed(1); coralOutake.setSpeed(1);}, liberator, coralOutake))
            //.onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));


        //Algae Pivot Position Bindings
        //operator.povLeft().onTrue(new AlgaeFromReefPivotCommand(pivot));
        //operator.povUp().onTrue(new AlgaePivotResetCommand(pivot));

        // Back Up Driving Features
        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */
        
        // Sys ID Routines
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        driver.triangle().and(driver.R1()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.triangle().and(driver.L1()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.square().and(driver.R1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.square().and(driver.L1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
