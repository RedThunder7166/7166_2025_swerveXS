// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.CoralStationID;
import frc.robot.subsystems.CameraSubsystem.RelativeReefLocation;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static RobotContainer singleton = null;

    public static RobotContainer getSingleton() {
        if (singleton == null)
            singleton = new RobotContainer();
        return singleton;
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric m_robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake m_brakeDrive = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;

    private final Command m_fieldCentricDriveCommand;
    private final Command m_robotCentricDriveCommand;
    private boolean m_isFieldCentric = true;

    private final Rotation2d m_initialSwerveRotation;

    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_driveSubsystem = TunerConstants.createDrivetrain();
        m_cameraSubsystem = CameraSubsystem.getSingleton();
        m_cameraSubsystem.setDriveSubsystem(m_driveSubsystem, MaxSpeed, MaxAngularRate);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_fieldCentricDriveCommand = m_driveSubsystem.applyRequest(() ->
            m_fieldCentricDrive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
        m_robotCentricDriveCommand = m_driveSubsystem.applyRequest(() ->
            m_robotCentricDrive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

        m_autoChooser = AutoBuilder.buildAutoChooser("driveStraight");
        SmartDashboard.putData("AutoChooser", m_autoChooser);

        // make sure forward faces red alliance wall
        if (Constants.alliance == Alliance.Red)
            m_initialSwerveRotation = Rotation2d.kZero;
        else
            m_initialSwerveRotation = Rotation2d.k180deg;

        m_driveSubsystem.resetCustomEstimatedRotation(m_initialSwerveRotation);

        m_driveSubsystem.ensureThisFileHasBeenModified();

        configureBindings();
    }

    public void setFieldCentric(boolean beFieldCentric) {
        m_isFieldCentric = beFieldCentric;
    }

    private RelativeReefLocation m_targetReefLocation = RelativeReefLocation.AB;
    private void configureBindings() {
        setFieldCentric(true);
        m_driveSubsystem.setDefaultCommand(m_driveSubsystem.applyRequest(() -> {
            return m_isFieldCentric ? m_fieldCentricDrive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
             : m_robotCentricDrive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            }
        ));

        m_joystick.leftBumper().whileTrue(m_driveSubsystem.applyRequest(() -> {
            return m_robotCentricDrive.withVelocityY(0.25 * MaxSpeed);
        }));
        m_joystick.rightBumper().whileTrue(m_driveSubsystem.applyRequest(() -> {
            return m_robotCentricDrive.withVelocityY(-0.25 * MaxSpeed);
        }));

        m_joystick.a().whileTrue(m_driveSubsystem.applyRequest(() -> m_brakeDrive));
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

        SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        m_joystick.povUp().onTrue(new InstantCommand(() -> {
            m_targetReefLocation = m_targetReefLocation.getNext();
            // if (m_targetReefLocation > 11)
            //     m_targetReefLocation = 6;
            SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        }));
        m_joystick.povDown().onTrue(new InstantCommand(() -> {
            m_targetReefLocation = m_targetReefLocation.getPrevious();
            // if (m_targetReefLocation < 6)
            //     m_targetReefLocation = 11;
            SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        }));

        // path plan to tag
        m_joystick.b().whileTrue(new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromReefTag(m_targetReefLocation);
        }));

        // path plan to left coral station
        m_joystick.povLeft().whileTrue(new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromCoralStationTag(CoralStationID.Left);
        }));
        // path plan to left coral station
        m_joystick.povRight().whileTrue(new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromCoralStationTag(CoralStationID.Right);
        }));

        // m_joystick.leftBumper().whileTrue(m_driveSubsystem.applyRequest(() ->
        //     drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //         .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //         .withRotationalRate(m_cameraSubsystem.calculateRotateFromTag(7)) // Drive counterclockwise with negative X (left)
        // ));

        // reset the field-centric heading
        m_joystick.start().onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.seedFieldCentric();
        }));

        // reset rotation to set rotation based on alliance
        m_joystick.back().onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.resetCustomEstimatedRotation(m_initialSwerveRotation);
        }));

        // Do NOT forget about the callback in the drive subsystem file (only one will be active).
        // m_driveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return m_autoChooser.getSelected();
    }
}
