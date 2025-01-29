// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    public static final int REEF_AB_TAGID = alliance == Alliance.Blue ? 18 : 7;
    public static final int REEF_CD_TAGID = alliance == Alliance.Blue ? 19 : 8;
    public static final int REEF_EF_TAGID = alliance == Alliance.Blue ? 20 : 9;
    public static final int REEF_GH_TAGID = alliance == Alliance.Blue ? 21 : 10;
    public static final int REEF_IJ_TAGID = alliance == Alliance.Blue ? 22 : 11;
    public static final int REEF_KL_TAGID = alliance == Alliance.Blue ? 17 : 6;

    public static final int CORAL_STATION_LEFT_TAGID = alliance == Alliance.Blue ? 13 : 1;
    public static final int CORAL_STATION_RIGHT_TAGID = alliance == Alliance.Blue ? 12 : 2;

    // below values are in meters
    public static final double INSIDE_REEF_ZONE_THRESHOLD = 1.6;

    private static final double CORAL_STATION_OFFSET_HORIZONTAL = 0.3;
    private static final double CORAL_STATION_OFFSET_VERTICAL = 0.3;
    public static final Translation2d CORAL_STATION_LEFT_OFFSET = alliance == Alliance.Blue ?
        new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
        new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
    public static final Translation2d CORAL_STATION_RIGHT_OFFSET = alliance == Alliance.Blue ?
        new Translation2d(-CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
        new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
}