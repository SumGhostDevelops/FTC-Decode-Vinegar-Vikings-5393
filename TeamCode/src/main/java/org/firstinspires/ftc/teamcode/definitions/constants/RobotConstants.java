package org.firstinspires.ftc.teamcode.definitions.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveMode;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;

public class RobotConstants
{
    @Configurable
    public static class General
    {
        public static ConstantsPresets.Preset PRESET_OPTION = ConstantsPresets.Preset.TESTING;
        public static boolean ENERGY_SAVER_MODE = false;
        public static boolean REGRESSION_TESTING_MODE = true;
        public static double MOTOR_ACCELERATION_FILTER_FACTOR = 0.8;
    }

    @Configurable
    public static class Telemetry
    {
        public static boolean SET_AUTOCLEAR = true;
        public static boolean SET_AUTOCLEAR_LOGS = true;
        public static double LOG_AUTOCLEAR_DELAY = 10;

        public static boolean ENABLE_FIELD_DRAWING = true;
        public static boolean ENABLE_GRAPH_OUTPUT = true;
    }

    public static class AprilTags
    {
        public static int[] GOAL_IDS = new int[]
        { 20, 24 };
        public static int[] OBELISK_IDS = new int[]
        { 21, 22, 23 };
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

        public static DriveMode DRIVE_MODE = DriveMode.RAW_ROBOT_CENTRIC;

        @Configurable
        public static class Speed
        {
            public static double DEFAULT = 1.0;
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
        public static final double RPM_WHILE_MOVING_RATIO = 0.8;
        @Sorter(sort = 0)
        public static int PPR = 28;
        @Sorter(sort = 1)
        public static double INPUT_GEAR_RATIO = 3;
        public static double OUTPUT_GEAR_RATIO = 2;

        @Sorter(sort = 2)
        public static int BASE_RPM = 3000;

        @Sorter(sort = 3)
        public static boolean ON_BY_DEFAULT = false;
        @Sorter(sort = 4)
        public static boolean AUTO_DISTANCE_ADJUSMENT = false;

        @Sorter(sort = 5)
        public static boolean USE_FUTURE_POSE = false;
        @Sorter(sort = 6)
        public static double FUTURE_POSE_TIME = 1.0;

        @IgnoreConfigurable
        public static class Name
        {
            public static String LAUNCHER_LEFT = "leftOuttake";
            public static String LAUNCHER_RIGHT = "rightOuttake";
        }

        @Configurable
        public static class Coefficients
        {
            public static PIDFCoefficients PIDF = new PIDFCoefficients(0.001, 0.000001, 0, 0.0002); // only change I
        }

        @Configurable
        public static class Tolerance
        {
            @Sorter(sort = 0)
            public static int RPM = 150;
            @Sorter(sort = 1)
            public static int RPM_ACCELERATION = 500;
        }
    }

    @Configurable
    public static class Turret
    {
        @Sorter(sort = 0)
        public static String NAME = "turret";
        @Sorter(sort = 1)
        public static double GEAR_RATIO = 4.55; // 19.2 is the gear ratio, 4.5 is the motor to lazysusan ratio
        @Sorter(sort = 2)
        public static Angle TOLERANCE = new Angle(2, AngleUnit.DEGREES); // in degrees
        public static Distance LINEAR_TOLERANCE = new Distance(3, DistanceUnit.INCH);
        @Sorter(sort = 3)
        public static boolean USE_DYNAMIC_TOLERANCE = true;
        @Sorter(sort = 4)
        public static Angle FORWARD_ANGLE = new Angle(0, AngleUnit.DEGREES);
        @Sorter(sort = 5)
        public static UnnormalizedAngle[] TURN_LIMITS = new UnnormalizedAngle[]
        {
                new UnnormalizedAngle(-260, UnnormalizedAngleUnit.DEGREES),
                new UnnormalizedAngle(100, UnnormalizedAngleUnit.DEGREES) }; // in both directions, so if 0 is forward
        @Sorter(sort = 6)
        public static boolean AUTO_AIM_TO_GOAL = false;

        @Sorter(sort = 7)
        // only change p and d
        public static PIDFCoefficients PIDF = new PIDFCoefficients(0.08, 0.00, 0.0007, 0.00);

        @Sorter(sort = 8)
        public static boolean USE_FUTURE_POSE = false;
        @Sorter(sort = 9)
        public static double FUTURE_POSE_TIME = 1.0;

        @Sorter(sort = 10)
        public static boolean ROTATION_COMPENSATION_ENABLED = true;
        @Sorter(sort = 11)
        public static double ROTATION_COMPENSATION_FF = 0.0006; // Feedforward gain: power per deg/s of robot rotation

        @Sorter(sort = 12)
        public static Angle SAFETY_MARGIN = new Angle(10, AngleUnit.DEGREES);
    }

    @Configurable
    public static class Transfer
    {
        @Sorter(sort = 0)
        public static String NAME = "transfer";

        @Sorter(sort = 1)
        public static double OPEN_ANGLE = 90; // Open means the transfer is allowing balls to pass through

        @Sorter(sort = 2)
        public static double CLOSE_INTAKE_ANGLE = 210; // 210; // An angle where the trapdoor blocks balls from entering

        @Sorter(sort = 3)
        public static double CLOSE_TRANSFER_ANGLE = 0;
    }

    @Configurable
    public static class Intake
    {
        @Sorter(sort = 0)
        public static String NAME = "intake";

        @Sorter(sort = 1)
        public static double intakePower = 0.85;
        @Sorter(sort = 2)
        public static double outtakePower = 0.6;
        @Sorter(sort = 3)
        public static double transferPower = 1.0;

        @Sorter(sort = 7)
        public static boolean INTAKE_BY_DEFAULT = false;

        public static PIDFCoefficients PIDF = new PIDFCoefficients(1, 1, 1, 0);
    }

    @Configurable
    public static class Odometry
    {
        @Sorter(sort = 0)
        public static Pose2d DEFAULT_POSE = new Pose2d(
                new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH),
                        CoordinateSystem.DECODE_PEDROPATH),
                new FieldHeading(new Angle(90, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH));
        @Sorter(sort = 1)
        public static double FUTURE_POSE_TIME = 1.0;

        public static boolean SET_FORWARD_DIRECTION_BASED_ON_TEAM = true;

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
                public static Distance Y = new Distance(-4, DistanceUnit.INCH); // camera forward of center, was 4.75
                @Sorter(sort = 2)
                public static Distance Z = new Distance(9, DistanceUnit.INCH);

                // Camera orientation (YawPitchRoll) - FTC SDK Convention:
                // Yaw: 0 = pointing forward, +90 = pointing left, -90 = pointing right
                // Pitch: 0 = pointing straight up, -90 = horizontal (forward), +90 = pointing
                // straight down
                // Roll: 0 = level, +/-90 = vertical, 180 = upside-down
                //
                // NOTE: If camera localization became inaccurate after hardware changes,
                // verify these values match the physical camera mounting orientation.
                // A slight tilt or rotation of the camera requires updating these values.
                @Sorter(sort = 0)
                public static Angle YAW = new Angle(0, AngleUnit.DEGREES);
                @Sorter(sort = 1)
                public static Angle PITCH = new Angle(0, AngleUnit.DEGREES); // horizontal camera pointing forward
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
                // not great but good enough
                public static Distance OFFSET = new Distance(-38, DistanceUnit.MM);
            }

            @Configurable
            public static class Strafe // also perpendicular
            {
                // Distance from the center to the STRAFE wheel along the X-axis (LONGITUDINAL)
                // Positive if FORWARD of center, Negative if BACKWARD of center
                // not great but good enough
                public static Distance OFFSET = new Distance(-5.93, DistanceUnit.INCH);
            }
        }
    }
}