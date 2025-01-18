// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class CameraSubsystem extends SubsystemBase {
    private static CameraSubsystem singleton = null;

    public static CameraSubsystem getSingleton() {
        if (singleton == null)
            singleton = new CameraSubsystem();
        return singleton;
    }

    private static final String limelightName = "limelight";

    private static final HashMap<Integer, AprilTag> aprilTagMap = new HashMap<>();
    static {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            for (var tag : fieldLayout.getTags()) {
                aprilTagMap.put(tag.ID, tag);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), false);
        }
    }

    private CommandSwerveDrivetrain m_driveSubsystem;
    private Pigeon2 m_pigeon2;
    public void setDriveSubsystem(CommandSwerveDrivetrain driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_pigeon2 = m_driveSubsystem.getPigeon2();
    }

    private boolean useMegaTag1 = true;

    public CameraSubsystem() {
    }

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
    private void updateVisionMegaTag1() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7)
                return;
            else if (mt1.rawFiducials[0].distToCamera > 3)
                return;
        } else if (mt1.tagCount == 0)
            return;

        m_driveSubsystem.setVisionMeasurementStdDevs(stdDevs);
        m_driveSubsystem.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds
        );
    }
    private void updateVisionMegaTag2() {
        var state = m_driveSubsystem.getState();
        LimelightHelpers.SetRobotOrientation("limelight", state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if(Math.abs(m_pigeon2.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            return;
        if(mt2.tagCount == 0)
            return;

        m_driveSubsystem.setVisionMeasurementStdDevs(stdDevs);
        m_driveSubsystem.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds
        );
    }

    @Override
    public void periodic() {
        if (useMegaTag1)
            updateVisionMegaTag1();
        else
            updateVisionMegaTag2();
    }
}
