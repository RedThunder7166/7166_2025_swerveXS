// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain m_driveSubsystem = TunerConstants.createDrivetrain();
    private final CameraSubsystem m_cameraSubsystem = CameraSubsystem.getSingleton();
    {
        m_cameraSubsystem.setDriveSubsystem(m_driveSubsystem, MaxSpeed, MaxAngularRate);
    }

    private final Rotation2d m_initialSwerveRotation;

    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_autoChooser = AutoBuilder.buildAutoChooser("driveStraight");
        SmartDashboard.putData("AutoChooser", m_autoChooser);

        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
        // for second limelight
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port + 10, "limelight-mini.local", port);
        }

        // make sure forward faces red alliance wall
        if (Constants.alliance == Alliance.Red)
            m_initialSwerveRotation = Rotation2d.kZero;
        else
            m_initialSwerveRotation = Rotation2d.k180deg;

        m_driveSubsystem.resetCustomRotation(m_initialSwerveRotation);

        m_driveSubsystem.ensureThisFileHasBeenModified();

        configureBindings();
    }

    int target_tagid = 7;
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_driveSubsystem.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_driveSubsystem.applyRequest(() ->
                drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_joystick.a().whileTrue(m_driveSubsystem.applyRequest(() -> brake));
        // m_joystick.b().whileTrue(m_driveSubsystem.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_joystick.getLeftY(), -m_joystick.getLeftX()))
        // ));

        // m_joystick.pov(0).whileTrue(m_driveSubsystem.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // m_joystick.pov(180).whileTrue(m_driveSubsystem.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // m_joystick.back().and(m_joystick.y()).whileTrue(m_driveSubsystem.sysIdDynamic(Direction.kForward));
        // m_joystick.back().and(m_joystick.x()).whileTrue(m_driveSubsystem.sysIdDynamic(Direction.kReverse));
        // m_joystick.start().and(m_joystick.y()).whileTrue(m_driveSubsystem.sysIdQuasistatic(Direction.kForward));
        // m_joystick.start().and(m_joystick.x()).whileTrue(m_driveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // robot-centric
        // m_joystick.rightBumper().whileTrue(m_driveSubsystem.applyRequest(() -> 
        //     forwardStraight.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //         .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //         .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));
        // drive and rotate toward tag
        // m_joystick.rightBumper().whileTrue(m_driveSubsystem.applyRequest(() ->
        //     forwardStraight.withVelocityX(m_cameraSubsystem.calculateRangeFromTag()) // Drive forward with negative Y (forward)
        //         .withVelocityY(0) // Drive left with negative X (left)
        //         .withRotationalRate(m_cameraSubsystem.calculateRotateFromTag()) // Drive counterclockwise with negative X (left)
        // ));

        SmartDashboard.putNumber("TARGET_TAGID", target_tagid);
        m_joystick.povUp().onTrue(new InstantCommand(() -> {
            target_tagid++;
            if (target_tagid > 11)
                target_tagid = 6;
            SmartDashboard.putNumber("TARGET_TAGID", target_tagid);
        }));
        m_joystick.povDown().onTrue(new InstantCommand(() -> {
            target_tagid--;
            if (target_tagid < 6)
                target_tagid = 11;
            SmartDashboard.putNumber("TARGET_TAGID", target_tagid);
        }));

        // path plan to tag
        m_joystick.rightBumper().whileTrue(new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromTag(target_tagid);
        }));

        m_joystick.leftBumper().whileTrue(m_driveSubsystem.applyRequest(() ->
            drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(m_cameraSubsystem.calculateRotateFromTag(7)) // Drive counterclockwise with negative X (left)
        ));

        // reset the field-centric heading
        m_joystick.start().onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.seedFieldCentric();
        }));

        // reset rotation to set rotation based on alliance
        m_joystick.back().onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.resetCustomRotation(m_initialSwerveRotation);
        }));

        m_driveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return m_autoChooser.getSelected();
    }
}
