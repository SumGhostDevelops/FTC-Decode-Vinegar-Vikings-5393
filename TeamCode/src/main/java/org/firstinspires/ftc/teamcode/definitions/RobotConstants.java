package org.firstinspires.ftc.teamcode.definitions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class RobotConstants
{
    // Drive Constants
    public static double DRIVE_SPEED_MULTIPLIER = 1.0;
    public final static double DRIVE_SPEED_MAXIMUM = 1;
    public final static double DRIVE_SPEED_MINIMUM = 0.25;
    public final static double DRIVE_SPEED_CHANGE = 0.25;

    public static double HYBRID_MODE_TURN_P = 0.02;
    public static double HYBRID_MODE_TURN_D = 0.001;
    public static double HYBRID_MODE_DEADBAND = 0.05;

    // Auto Aim Constants
    public static double FORCED_ANGLE_OFFSET = 3;

    // Outtake Constants
    public final static int OUTTAKE_PPR = 28;
    public final static int OUTTAKE_INITIAL_TARGET_RPM = 4150; // Tune this
    public static int OUTTAKE_RPM_TOLERANCE = 100;
    public static int OUTTAKE_STABILITY_TOLERANCE = 150;
    public static double SHOT_DROP_THRESHOLD = 0.20; // 20% drop implies shot
    public static double OUTTAKE_F_OFFSET = 0.0;

    // Camera Offsets
    public final static double cameraOffsetX = 0.0;
    public final static double cameraOffsetY = 0.0;

    public static boolean TELEMETRY_SET_AUTOCLEAR = true;

    public static class AprilTags
    {
        public final static int[] GOAL_IDS = new int[]{20, 24};
        public final static int[] OBELISK_IDS = new int[]{21, 22, 23};
    }

    // Hardware Names
    public static class Drive
    {
        public final static String FRONT_LEFT = "frontLeft";
        public final static String FRONT_RIGHT = "frontRight";
        public final static String BACK_LEFT = "backLeft";
        public final static String BACK_RIGHT = "backRight";
    }

    public static class Outtake
    {
        // Outtake Motors
        public final static int PPR = 28;
        public final static String LAUNCHER_LEFT = "leftOuttake";
        public final static String LAUNCHER_RIGHT = "rightOuttake";
        public final static String TURRET = "turret";
        public final static double[] veloCoeffs = new double[]{1.0, 1.0, 1.0};
        public final static double[] ffCoeffs = new double[]{1.0, 1.0, 1.0};
    }

    public static class Transfer
    {
        public final static String TRANSFER = "transfer";
    }

    public static class Intake
    {
        public final static String INTAKE = "intake";
    }

    public static class Odometry
    {
        public static class Pinpoint
        {
            public final static String PINPOINT = "pinpoint";
            public final static double xOffset = 0;
            public final static double yOffset = 0;
            public final static DistanceUnit offsetUnit = DistanceUnit.INCH;
        }
        public static class Webcam
        {

            public final static String WEBCAM = "webcam";
            public final static double LENS_FX = 1424.38;
            public final static double LENS_FY = 1424.38;
            public final static double LENS_CX = 637.325;
            public final static double LENS_CY = 256.774;
        }

        public static class Deadwheels // These are plugged in directly to the Pinpoint
        {
            public final static int COUNTS_PER_REVOLUTION = 8192;
            public final static int WHEEL_CIRCUMFERENCE = 35; // fix this number
            public final static DistanceUnit circumferenceUnit = DistanceUnit.MM;
        }
    }
}