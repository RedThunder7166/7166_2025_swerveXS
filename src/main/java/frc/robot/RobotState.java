// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    public static Alliance currentAlliance = Alliance.Red;
    static {
        try {
            var optional = DriverStation.getAlliance();
            if (optional.isEmpty()) {
                throw new Exception("Failed to get alliance from driver station; defaulting to Red");
            }
            currentAlliance = optional.get();
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
    }

    public static double swerveHeadingOffset = 0;
}
