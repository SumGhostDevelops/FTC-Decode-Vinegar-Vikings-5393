package org.firstinspires.ftc.teamcode.definitions;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants
{
    // Drive Constants
    public static double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static double DRIVE_SPEED_MAXIMUM = 1;
    public static double DRIVE_SPEED_MINIMUM = 0.25;
    public static double DRIVE_SPEED_CHANGE = 0.25;

    public static double HYBRID_MODE_TURN_P = 0.02;
    public static double HYBRID_MODE_TURN_D = 0.001;
    public static double HYBRID_MODE_DEADBAND = 0.05;

    // Auto Aim Constants
    public static double FORCED_ANGLE_OFFSET = 3;

    // Outtake Constants
    public static int OUTTAKE_PPR = 28;
    public static double OUTTAKE_POWER_FWD = 1.0;
    public static double OUTTAKE_POWER_REV = -0.7;
    public static double OUTTAKE_POWER_IDLE = 0.0;
    public static int OUTTAKE_INITIAL_TARGET_RPM = 3900; // Tune this
    public static int OUTTAKE_RPM_TOLERANCE = 30;
    public static int OUTTAKE_STABILITY_TOLERANCE = 250;
    public static double SHOT_DROP_THRESHOLD = 0.20; // 20% drop implies shot
    public static double JAM_CLEAR_POWER = -0.3;
    public static double JAM_CLEAR_DURATION = 1.0; // Seconds

    // Transfer Constants
    public static double TRANSFER_POWER_FWD = 0.8;
    public static double TRANSFER_POWER_REV = -0.8;
    public static double TRANSFER_POWER_IDLE = 0.0;

    // Localization / Vision
    public static double LENS_FX = 1424.38;
    public static double LENS_FY = 1424.38;
    public static double LENS_CX = 637.325;
    public static double LENS_CY = 256.774;

    // Dead Wheels (Two Wheel Config)
    public static double PAR_Y_TICKS = 0.0; // Forward/Back offset
    public static double PERP_X_TICKS = 0.0; // Left/Right offset
    public static double IN_PER_TICK = 0.001; // Tune this

    // Camera Offsets
    public static double cameraOffsetX = 0.0;
    public static double cameraOffsetY = 0.0;
}