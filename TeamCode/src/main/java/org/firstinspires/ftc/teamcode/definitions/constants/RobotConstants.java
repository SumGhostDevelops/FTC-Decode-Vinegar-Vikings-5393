package org.firstinspires.ftc.teamcode.definitions.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveMode;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;

public class RobotConstants
{
    @Configurable
    public static class General
    {
        public static ConstantsPresets.Preset PRESET_OPTION = ConstantsPresets.Preset.TESTING;
        public static boolean ENERGY_SAVER_MODE = false;
        public static boolean REGRESSION_TESTING_MODE = false;
    }

    @Configurable
    public static class Telemetry
    {
        public static boolean SET_AUTOCLEAR = true;
        public static boolean SET_AUTOCLEAR_LOGS = true;
    }

    public static class AprilTags
    {
        public static int[] GOAL_IDS = new int[]{20, 24};
        public static int[] OBELISK_IDS = new int[]{21, 22, 23};
    }

    @Configurable
    // Hardware Names
    public static class Drive
    {
        @IgnoreConfigurable
        public static class WHEEL_NAMES
        {
            public static String FRONT_LEFT = "frontLeft";
            public static String FRONT_RIGHT = "frontRight";
            public static String BACK_LEFT = "backLeft";
            public static String BACK_RIGHT = "backRight";
        }

        public DriveMode driveMode = org.firstinspires.ftc.teamcode.subsystems.Drive.DriveMode.FIELD_CENTRIC;

        @Configurable
        public static class Speed
        {
            public static double DEFAULT = 0.75;
            public static double MAXIMUM = 1.0;
            public static double MINIMUM = 0.25;
            public static double CHANGE = 0.25;
        }

        @IgnoreConfigurable
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
        @Sorter(sort = 0)
        public static int PPR = 28;
        @Sorter(sort = 1)
        public static double GEAR_RATIO = 1.5; // driver::driving 3::2

        @Sorter(sort = 2)
        public static int BASE_RPM = 3700;

        @Sorter(sort = 3)
        public static boolean IDLE_BY_DEFAULT = false;
        @Sorter(sort = 4)
        public static boolean AUTO_DISTANCE_ADJUSMENT = true;

        @IgnoreConfigurable
        public static class Name
        {
            public static String LAUNCHER_LEFT = "leftOuttake";
            public static String LAUNCHER_RIGHT = "rightOuttake";
        }

        @Configurable
        public static class Coefficients
        {
            public static double[] veloCoeffs = new double[]{10, 3, 2, 0};
        }

        @Configurable
        public static class Tolerance
        {
            @Sorter(sort = 0)
            public static int RPM = 75;
            @Sorter(sort = 1)
            public static int RPM_ACCELERATION = 100;
        }
    }

    @Configurable
    public static class Turret
    {
        @Sorter(sort = 0)
        public static String NAME = "turret";

        @Sorter(sort = 1)
        public static double PPR = 537.7;
        @Sorter(sort = 2)
        public static double GEAR_RATIO = 19.2 * 4.5; // 19.2 is the gear ratio, 4.5 is the motor to lazysusan ratio
        @Sorter(sort = 3)
        public static int TOLERANCE = 5; // in ticks

        @Sorter(sort = 4)
        public static Angle FORWARD_ANGLE = new Angle(0, AngleUnit.DEGREES);
        @Sorter(sort = 5)
        public static UnnormalizedAngle[] TURN_LIMITS = new UnnormalizedAngle[]{new UnnormalizedAngle(-270, UnnormalizedAngleUnit.DEGREES), new UnnormalizedAngle(90, UnnormalizedAngleUnit.DEGREES)}; // in both directions, so if 0 is forward
        @Sorter(sort = 6)
        public static boolean autoAimToGoal = true;

        @Sorter(sort = 7)
        public static double pCoeff = 1;
    }

    @Configurable
    public static class Transfer
    {
        @Sorter(sort = 0)
        public static String NAME = "transfer";

        @Sorter(sort = 1)
        public static double SERVO_RANGE = 360.0; // Physical servo range in degrees
        @Sorter(sort = 2)
        public static double OPEN_ANGLE = 75; // Open means the transfer is allowing balls to pass through
        @Sorter(sort = 3)
        public static double CLOSED_INTAKE_ANGLE = 0; // An angle where the trapdoor blocks balls from entering
        @Sorter(sort = 4)
        public static double CLOSED_FULL_TRANSFER_ANGLE = 210;

        public static double CLOSED_SHOOTING_TRANSFER_ANGLE = 60;
        @Sorter(sort = 5)
        public static double TRANSFER_ANGLE = 100;

        @Sorter(sort = 6)
        public static double autoCloseMs = 500; // After the outtake goes from ready -> not ready, the transfer will automatically close for this length.

        @Sorter(sort = 7)
        public static boolean autoTransferPrevent = false;
        @Sorter(sort = 8)
        public static boolean RELEASE_ALL_BALLS_WHEN_READY = false;

        @Configurable
        public static class TimerConstants
        {
            @Sorter(sort = 0)
            public static int totalTime = 600;
            @Sorter(sort = 1)
            public static int upTime = 300;
            @Sorter(sort = 2)
            public static int downTime = 300;
        }
    }

    @Configurable
    public static class Intake
    {
        @Sorter(sort = 0)
        public static String NAME = "intake";

        @Sorter(sort = 1)
        public static double intakeRPM = 1300;
        @Sorter(sort = 2)
        public static double outtakePower = 1300;
        @Sorter(sort = 3)
        public static double transferPassRPM = 1620;
        @Sorter(sort = 4)
        public static double transferPassPowerSpeed = 0.5;
        @Sorter(sort = 5)
        public static double transferPreventPower = 1.0;

        @Sorter(sort = 6)
        public static double transferPreventDurationMs = 100;

        @Sorter(sort = 7)
        public static boolean INTAKE_BY_DEFAULT = false;
    }

    @Configurable
    public static class Odometry
    {
        @Sorter(sort = 0)
        public static Pose2d DEFAULT_POSE = new Pose2d(new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH), FieldCoordinate.CoordinateSystem.RIGHT_HAND), new Angle(90, AngleUnit.DEGREES));
        @Sorter(sort = 1)
        public static double FUTURE_POSE_TIME = 1.0;

        @IgnoreConfigurable
        public static class IMU
        {
            public static String NAME = "imu";
        }

        @IgnoreConfigurable
        public static class Pinpoint
        {
            public static String NAME = "pinpoint";
        }

        @IgnoreConfigurable
        public static class Webcam
        {
            public static String NAME = "webcam";

            @Configurable
            public static class Lens
            {

                public static double LENS_FX = 958.876;
                public static double LENS_FY = 958.876;
                public static double LENS_CX = 654.11;
                public static double LENS_CY = 358.336;
            }

            @Configurable
            public static class Offset
            {
                // Position of camera relative to robot center
                // Uses FTC SDK conventions for setCameraPose():
                // X: Left/right (positive = RIGHT of center)
                // Y: Forward/backward (positive = FORWARD of center)
                // Z: Height above ground
                @Sorter(sort = 0)
                public static Distance X = new Distance(0, DistanceUnit.INCH); // camera centered left/right
                @Sorter(sort = 1)
                public static Distance Y = new Distance(4.75, DistanceUnit.INCH); // camera forward of center
                @Sorter(sort = 2)
                public static Distance Z = new Distance(9, DistanceUnit.INCH);

                // Camera orientation (YawPitchRoll)
                // Yaw: 0 = pointing forward, +90 = pointing left, -90 = pointing right
                // Pitch: -90 = horizontal (pointing forward), 0 = pointing straight up
                // Roll: 0 = level, +/-90 = vertical, 180 = upside-down
                @Sorter(sort = 0)
                public static Angle YAW = new Angle(0, AngleUnit.DEGREES);
                @Sorter(sort = 1)
                public static Angle PITCH = new Angle(-90, AngleUnit.DEGREES); // horizontal camera
                @Sorter(sort = 2)
                public static Angle ROLL = new Angle(0, AngleUnit.DEGREES);
            }
        }

        @Configurable
        public static class Deadwheels // These are plugged in directly to the Pinpoint
        {
            @Sorter(sort = 0)
            public static int COUNTS_PER_REVOLUTION = 8192;

            @Sorter(sort = 1)
            public static Distance WHEEL_DIAMETER = new Distance(35, DistanceUnit.MM);
            @Sorter(sort = 2)
            public static Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.multiply(Math.PI); // π × diameter

            @Configurable
            public static class Forward // also parallel
            {
                // Distance from the center to the FORWARD wheel along the Y-axis (LATERAL)
                // Positive if LEFT of center, Negative if RIGHT of center
                public static Distance OFFSET = new Distance(6.68, DistanceUnit.INCH);
            }

            @Configurable
            public static class Strafe // also perpendicular
            {
                // Distance from the center to the STRAFE wheel along the X-axis (LONGITUDINAL)
                // Positive if FORWARD of center, Negative if BACKWARD of center
                public static Distance OFFSET = new Distance(-7.81, DistanceUnit.INCH);
            }
        }
    }
}