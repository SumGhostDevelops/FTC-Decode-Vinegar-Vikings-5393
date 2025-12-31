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
    @Config
    public static class Telemetry
    {
        public static boolean SET_AUTOCLEAR = true;
    }

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

        @Config
        public static class Speed
        {
            public final static double DEFAULT = 1.0;
            public final static double MAXIMUM = 1.0;
            public final static double MINIMUM = 0.25;
            public final static double CHANGE = 0.25;
        }

        @Config
        public static class HybridMode
        {
            public static double TURN_P = 0.02;
            public static double TURN_D = 0.001;
            public static double DEADBAND = 0.05;
        }
    }

    @Config
    public static class Outtake
    {
        public final static int PPR = 28;

        public static class Name
        {
            public final static String LAUNCHER_LEFT = "leftOuttake";
            public final static String LAUNCHER_RIGHT = "rightOuttake";
        }

        public static class Coefficients
        {

            public final static double[] veloCoeffs = new double[]{1.0, 1.0, 1.0};
            public final static double[] ffCoeffs = new double[]{1.0, 1.0, 1.0};
        }

        public static class Tolerance
        {
            public static int RPM = 100;
            public static int RPM_ACCELERATION = 150;
        }
    }

    @Config
    public static class Turret
    {
        public final static String TURRET = "turret";
        public final static int PPR = 28;
        public final static int TOLERANCE = 0; // in ticks
        public final static double GEAR_RATIO = 19.2 * 4.5; // 19.2 is the gear ratio, 4.5 is the motor to lazysusan ratio
        public final static Angle FORWARD_ANGLE = new Angle(90, AngleUnit.DEGREES);
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
        public final static String NAME = "intake";
    }

    @Config
    public static class Odometry
    {
        public final static Pose2d DEFAULT_POSE = new Pose2d(new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(90, AngleUnit.DEGREES));
        @Config
        public static class Pinpoint
        {
            public final static String NAME = "pinpoint";
        }

        @Config
        public static class Webcam
        {

            public final static String NAME = "webcam";

            public static class Lens
            {

                public final static double LENS_FX = 1424.38;
                public final static double LENS_FY = 1424.38;
                public final static double LENS_CX = 637.325;
                public final static double LENS_CY = 256.774;
            }

            public static class Offset
            {
                public final static double X = 0.0;
                public final static double Y = 0.0;
            }
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