// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;

public class CameraSubsystem extends SubsystemBase {
    private static CameraSubsystem singleton = null;

    public static CameraSubsystem getSingleton() {
        if (singleton == null)
            singleton = new CameraSubsystem();
        return singleton;
    }

    private static final String limelightName = "limelight";

    private static final HashMap<Integer, AprilTag> aprilTagMap = new HashMap<>();
    private static Translation2d reefCenterTranslation = new Translation2d();
    static {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            for (var tag : fieldLayout.getTags()) {
                aprilTagMap.put(tag.ID, tag);
            }

            if (Constants.alliance == Alliance.Red) {
                var translation6 = aprilTagMap.get(6).pose.getTranslation();
                var translation7 = aprilTagMap.get(7).pose.getTranslation();
                var translation8 = aprilTagMap.get(8).pose.getTranslation();
                var translation9 = aprilTagMap.get(9).pose.getTranslation();
                var translation10 = aprilTagMap.get(10).pose.getTranslation();
                var translation11 = aprilTagMap.get(11).pose.getTranslation();
                reefCenterTranslation = new Translation2d(
                    (translation6.getX() + translation7.getX() + translation8.getX() + translation9.getX() + translation10.getX() + translation11.getX()) / 6,
                    (translation6.getY() + translation7.getY() + translation8.getY() + translation9.getY() + translation10.getY() + translation11.getY()) / 6
                );
            } else if (Constants.alliance == Alliance.Blue) {
                var translation17 = aprilTagMap.get(17).pose.getTranslation();
                var translation18 = aprilTagMap.get(18).pose.getTranslation();
                var translation19 = aprilTagMap.get(19).pose.getTranslation();
                var translation20 = aprilTagMap.get(20).pose.getTranslation();
                var translation21 = aprilTagMap.get(121).pose.getTranslation();
                var translation22 = aprilTagMap.get(122).pose.getTranslation();
                reefCenterTranslation = new Translation2d(
                    (translation17.getX() + translation18.getX() + translation19.getX() + translation20.getX() + translation21.getX() + translation22.getX()) / 6,
                    (translation17.getY() + translation18.getY() + translation19.getY() + translation20.getY() + translation21.getY() + translation22.getY()) / 6
                );
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), false);
        }
    }

    private CommandSwerveDrivetrain m_driveSubsystem;
    private Pigeon2 m_pigeon2;
    private double m_driveMaxSpeed;
    private double m_driveMaxAngularRate;
    public void setDriveSubsystem(CommandSwerveDrivetrain driveSubsystem, double driveMaxSpeed, double driveMaxAngularRate) {
        m_driveSubsystem = driveSubsystem;
        m_pigeon2 = m_driveSubsystem.getPigeon2();
        m_driveMaxSpeed = driveMaxSpeed;
        m_driveMaxAngularRate = driveMaxAngularRate;
    }

    private static final double speedMultiplier = 2.5;
    private static final double rotatePID_P = 0.027;
    private static final double rangePID_P = 0.065;
    private static final double targetTagRange = -12.5;

    private boolean useMegaTag2 = true;
    private boolean tunePIDWithSmartDashboard = false;

    public CameraSubsystem() {
        if (tunePIDWithSmartDashboard) {
            SmartDashboard.putNumber("DRIVE_ROTATE_P", rotatePID_P);
            SmartDashboard.putNumber("DRIVE_RANGE_P", rangePID_P);
        }
    }

    public double calculateRotateFromTag() {
        double kP = rotatePID_P;

        if (tunePIDWithSmartDashboard) {
            kP = SmartDashboard.getNumber("DRIVE_ROTATE_P", rotatePID_P);
            if (kP > 0.5)
                kP = 0.5;
            else if (kP < -0.5)
                kP = -0.5;
        }

        double targetingAngularVelocity = LimelightHelpers.getTX(limelightName) * kP;
        targetingAngularVelocity *= m_driveMaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    private final PIDController limelightRangeController = new PIDController(rangePID_P, 0, 0);
    public double calculateRangeFromTag() {    
        double kP = rangePID_P;

        if (tunePIDWithSmartDashboard) {
            kP = SmartDashboard.getNumber("DRIVE_RANGE_P", rangePID_P);
            if (kP > 0.5)
                kP = 0.5;
            else if (kP < -0.5)
                kP = -0.5;
        }

        limelightRangeController.setP(kP);
        double targetingForwardSpeed = limelightRangeController.calculate(LimelightHelpers.getTY(limelightName), targetTagRange);
        targetingForwardSpeed *= m_driveMaxSpeed * speedMultiplier;
        // targetingForwardSpeed *= -1.0;
        if (targetingForwardSpeed < 0)
            targetingForwardSpeed = 0;
        return targetingForwardSpeed;
    }

    private Pose2d latestVisionPose = null;
    public Pose2d getLatestVisionPose() {
        return latestVisionPose;
    }

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
    private void updateVisionMegaTag1() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null)
            return;

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7)
                return;
            else if (mt1.rawFiducials[0].distToCamera > 3)
                return;
        } else if (mt1.tagCount == 0)
            return;

        latestVisionPose = mt1.pose;

        m_driveSubsystem.setVisionMeasurementStdDevs(stdDevs);
        m_driveSubsystem.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds
        );
    }
    private boolean updateVisionMegaTag2() {
        var state = m_driveSubsystem.getState();
        double yaw_degrees = state.Pose.getRotation().minus(new Rotation2d(Angle.ofBaseUnits(RobotState.swerveHeadingOffset, Degrees))).getDegrees();
        SmartDashboard.putNumber("YAW_DEGREES", yaw_degrees);
        LimelightHelpers.SetRobotOrientation(limelightName, yaw_degrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null)
            return false;

        if(Math.abs(m_pigeon2.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            return false;
        if(mt2.tagCount == 0)
            return false;

        Pose2d pose = mt2.pose;
        latestVisionPose = pose;

        m_driveSubsystem.addVisionMeasurement(
            pose, mt2.timestampSeconds,
            stdDevs
        );

        return true;
    }

    StructPublisher<Pose2d> swervePosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
    @Override
    public void periodic() {
        if (useMegaTag2)
            SmartDashboard.putBoolean("MegaTag2Success", updateVisionMegaTag2());
        else
            updateVisionMegaTag1();

        var state = m_driveSubsystem.getState();
        swervePosePublisher.set(state.Pose);

        SmartDashboard.putNumber("StateX", state.Pose.getX());
        SmartDashboard.putNumber("StateY", state.Pose.getY());
        Pose2d visionPose = latestVisionPose;
        if (visionPose != null) {
            SmartDashboard.putNumber("VisionX", visionPose.getX());
            SmartDashboard.putNumber("VisionY", visionPose.getY());
        }
    }

    private final PIDController targetRotatePIDController = new PIDController(2, 0, 0);
    public double calculateRotateFromTag(int tagID) {
        // var state = m_driveSubsystem.getState();
        // Pose2d robotPose = state.Pose;
        Pose2d robotPose = latestVisionPose;
        if (robotPose == null)
            return 0;

        AprilTag targetTag = aprilTagMap.get(tagID);
        Pose2d targetTagPose = targetTag.pose.toPose2d();

        // var directionVector = targetTagPose.minus(robotPose);
        double desiredAngle = Math.atan2(targetTagPose.getY() - robotPose.getY(), targetTagPose.getX() - robotPose.getX());
        // double desiredAngle = directionVector.getRotation().getRadians();

        // double result = targetRotatePIDController.calculate(MathUtil.angleModulus(state.RawHeading.getRadians()), desiredAngle);
        double result = targetRotatePIDController.calculate(desiredAngle, 0);
        // result = -result;
        SmartDashboard.putNumber("ROTATEFROMTAG_RESULT", result);
        return result;
    }

    public static final class CommandWrapper extends Command {
        private final Supplier<Command> m_commandSupplier;
        private Command m_command;
        public CommandWrapper(Supplier<Command> commandSupplier) {
            m_commandSupplier = commandSupplier;
        }

        @Override
        public void initialize() {
            m_command = m_commandSupplier.get();
            m_command.schedule();
        }
        @Override
        public boolean isFinished() {
            return m_command.isFinished();
        }
        @Override
        public void end(boolean interuppted) {
            if (m_command.isScheduled())
                m_command.cancel();
        }
    }

    public Optional<Command> getPathCommandFromTag(int tagID) {
        Pose2d robotPose = latestVisionPose;

        // 29 inches
        double offset = Units.inchesToMeters(29);

        // this code gets the target april tag position and applies a certain offset away from the reef
        final AprilTag targetTag = aprilTagMap.get(tagID);
        final Pose2d targetTagPose = targetTag.pose.toPose2d();
        final Translation2d targetTagTranslation = targetTagPose.getTranslation();
        Translation2d directionVector = targetTagTranslation.minus(reefCenterTranslation); // get vector from center of reef to tag
        double directionVectorMagnitude = Math.sqrt(Math.pow(directionVector.getX(), 2) + Math.pow(directionVector.getY(), 2)); // get magnitude
        directionVector = new Translation2d(directionVector.getX() / directionVectorMagnitude, directionVector.getY() / directionVectorMagnitude); // normalize

        Pose2d targetPose = new Pose2d(
            new Translation2d(targetTagTranslation.getX() + directionVector.getX() * offset, targetTagTranslation.getY() + directionVector.getY() * offset),
            robotPose.getRotation()
        );

        SmartDashboard.putNumber("TagX", targetTagPose.getX());
        SmartDashboard.putNumber("TagY", targetTagPose.getY());

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                // 3.0, 4.0,
                1, 2,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command result = AutoBuilder.pathfindToPose(targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );

        result.addRequirements(m_driveSubsystem);
        return Optional.of(result);
    }
}
