package org.firstinspires.ftc.teamcode.definitions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;

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

    @Config
    public static class AprilTags
    {
        public final static int[] GOAL_IDS = new int[]{20, 24};
        public final static int[] OBELISK_IDS = new int[]{21, 22, 23};
    }

    // Hardware Names
    @Config
    public static class Drive
    {
        public final static String FRONT_LEFT = "frontLeft";
        public final static String FRONT_RIGHT = "frontRight";
        public final static String BACK_LEFT = "backLeft";
        public final static String BACK_RIGHT = "backRight";
    }

    @Config
    public static class Outtake
    {
        // Outtake Motors
        public final static int PPR = 28;
        public final static String LAUNCHER_LEFT = "leftOuttake";
        public final static String LAUNCHER_RIGHT = "rightOuttake";
        public final static double[] veloCoeffs = new double[]{1.0, 1.0, 1.0};
        public final static double[] ffCoeffs = new double[]{1.0, 1.0, 1.0};
    }

    @Config
    public static class Turret
    {
        public final static String TURRET = "turret";
        public final static int PPR = 28;
        public final static int TOLERANCE = 0; // in ticks
        public final static double GEAR_RATIO = 19.2 * 4.5; // 19.2 is the gear ratio, 4.5 is the motor to lazysusan ratio
        public final static Angle FORWARD_ANGLE = new Angle(0, AngleUnit.DEGREES); // should be the same as the IMU
        public final static UnnormalizedAngle[] TURN_LIMITS = new UnnormalizedAngle[]{new UnnormalizedAngle(-180, UnnormalizedAngleUnit.DEGREES), new UnnormalizedAngle(180, UnnormalizedAngleUnit.DEGREES)}; // in both directions, so if 0 is forward
        public final static double posCoeff = 1.0;
        public final static double[] ffCoeffs = new double[]{1.0, 1.0, 1.0};

        @Config
        public static class LazySusan
        {
            public final static Distance radius = new Distance(1.0, DistanceUnit.CM);
            public final static Distance circumference = radius.multiply(2 * Math.PI);
        }
    }

    @Config
    public static class Transfer
    {
        public final static String TRANSFER = "transfer";
        public final static double SERVO_RANGE = 360.0; // Physical servo range in degrees
        public final static double MIN_ANGLE = 0;
        public final static double MID_ANGLE = 30;
        public final static double MAX_ANGLE = 60;
    }

    @Config
    public static class Intake
    {
        public final static String INTAKE = "intake";
    }

    @Config
    public static class Odometry
    {
        public final static Pose2d DEFAULT_POSE = new Pose2d(new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(90, AngleUnit.DEGREES));
        @Config
        public static class Pinpoint
        {
            public final static String PINPOINT = "pinpoint";
        }

        @Config
        public static class Webcam
        {

            public final static String WEBCAM = "webcam";
            public final static double LENS_FX = 1424.38;
            public final static double LENS_FY = 1424.38;
            public final static double LENS_CX = 637.325;
            public final static double LENS_CY = 256.774;
        }

        @Config
        public static class Deadwheels // These are plugged in directly to the Pinpoint
        {
            public final static int COUNTS_PER_REVOLUTION = 8192;
            public final static Distance WHEEL_CIRCUMFERENCE = new Distance(35, DistanceUnit.MM);

            @Config
            public static class Forward
            {
                public final static Distance OFFSET = new Distance(0, DistanceUnit.INCH);
            }

            @Config
            public static class Strafe
            {
                public final static Distance OFFSET = new Distance(0, DistanceUnit.INCH);
            }
        }
    }
}