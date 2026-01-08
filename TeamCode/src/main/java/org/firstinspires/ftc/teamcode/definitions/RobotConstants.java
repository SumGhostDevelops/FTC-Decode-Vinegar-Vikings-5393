package org.firstinspires.ftc.teamcode.definitions;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;

public class RobotConstants
{

    @Configurable
    public static class Telemetry
    {
        public static boolean SET_AUTOCLEAR = true;
    }

    public static class AprilTags
    {
        public static int[] GOAL_IDS = new int[]{20, 24};
        public static int[] OBELISK_IDS = new int[]{21, 22, 23};
    }

    // Hardware Names
    public static class Drive
    {
        public static String FRONT_LEFT = "frontLeft";
        public static String FRONT_RIGHT = "frontRight";
        public static String BACK_LEFT = "backLeft";
        public static String BACK_RIGHT = "backRight";

        public static class Speed
        {
            public static double DEFAULT = 1.0;
            public static double MAXIMUM = 1.0;
            public static double MINIMUM = 0.25;
            public static double CHANGE = 0.25;
        }

        public static class HybridMode
        {
            public static double TURN_P = 0.02;
            public static double TURN_D = 0.001;
            public static double DEADBAND = 0.05;
        }
    }

    @Configurable
    public static class Outtake
    {
        public static int PPR = 28;
        public static int BASE_RPM = 4000;
        public static boolean IDLE_WHEN_END = false;

        public static class Name
        {
            public static String LAUNCHER_LEFT = "leftOuttake";
            public static String LAUNCHER_RIGHT = "rightOuttake";
        }

        @Configurable
        public static class Coefficients
        {
            public static double[] veloCoeffs = new double[]{1.0, 1.0, 1.0, 1.0};
        }

        @Configurable
        public static class Tolerance
        {
            public static int RPM = 75;
            public static int RPM_ACCELERATION = 100;
        }
    }

    @Configurable
    public static class Turret
    {
        public static String TURRET = "turret";
        public static int PPR = 28;
        public static int TOLERANCE = 0; // in ticks
        public static double GEAR_RATIO = 19.2 * 4.5; // 19.2 is the gear ratio, 4.5 is the motor to lazysusan ratio
        public static Angle FORWARD_ANGLE = new Angle(0, AngleUnit.DEGREES);
        public static UnnormalizedAngle[] TURN_LIMITS = new UnnormalizedAngle[]{new UnnormalizedAngle(-180, UnnormalizedAngleUnit.DEGREES), new UnnormalizedAngle(180, UnnormalizedAngleUnit.DEGREES)}; // in both directions, so if 0 is forward
        public static double posCoeff = 1.0;
        public static double[] ffCoeffs = new double[]{1.0, 1.0, 1.0};

        public static class LazySusan
        {
            public static Distance radius = new Distance(1.0, DistanceUnit.CM);
            public static Distance circumference = radius.multiply(2 * Math.PI);
        }
    }

    @Configurable
    public static class Transfer
    {
        public static String TRANSFER = "transfer";
        public static double SERVO_RANGE = 360.0; // Physical servo range in degrees
        public static double OPEN_ANGLE = 75; // Open means the transfer is allowing balls to pass through
        public static double MID_ANGLE = 30;
        public static double CLOSED_ANGLE = 0;
        public static double autoPauseMs = 1000; // After the outtake goes from ready -> not ready, the transfer will automatically close for this length.
    }

    @Configurable
    public static class Intake
    {
        public static String NAME = "intake";
        /**
         * If true, the intake will automatically be set to intake mode.
         */
        public static boolean automaticBehavior = true;
        public static double intakePower = 1.0;
        public static double transferPower = 1.0;
    }

    @Configurable
    public static class Odometry
    {
        public static Pose2d DEFAULT_POSE = new Pose2d(new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(90, AngleUnit.DEGREES));

        public static class IMU
        {
            public static String NAME = "imu";
        }

        public static class Pinpoint
        {
            public static String NAME = "pinpoint";
        }

        public static class Webcam
        {

            public static String NAME = "webcam";

            public static class Lens
            {

                public static double LENS_FX = 1424.38;
                public static double LENS_FY = 1424.38;
                public static double LENS_CX = 637.325;
                public static double LENS_CY = 256.774;
            }

            @Configurable
            public static class Offset
            {
                public static double X = 0.0;
                public static double Y = 0.0;
            }
        }

        @Configurable
        public static class Deadwheels // These are plugged in directly to the Pinpoint
        {
            public static int COUNTS_PER_REVOLUTION = 8192;
            public static Distance WHEEL_DIAMETER = new Distance(35, DistanceUnit.MM);
            public static Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.multiply(Math.PI); // π × diameter

            @Configurable
            public static class Forward // also parallel
            {
                public static String NAME = "forward";
                // Distance from the center to the FORWARD wheel along the Y-axis (LATERAL)
                // Positive if LEFT of center, Negative if RIGHT of center
                public static Distance OFFSET = new Distance(6.68, DistanceUnit.INCH);
            }

            @Configurable
            public static class Strafe // also perpendicular
            {
                public static String NAME = "strafe";
                // Distance from the center to the STRAFE wheel along the X-axis (LONGITUDINAL)
                // Positive if FORWARD of center, Negative if BACKWARD of center
                public static Distance OFFSET = new Distance(-7.81, DistanceUnit.INCH);
            }
        }
    }
}