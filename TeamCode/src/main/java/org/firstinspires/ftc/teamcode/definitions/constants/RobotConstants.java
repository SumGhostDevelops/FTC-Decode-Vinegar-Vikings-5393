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
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.UnnormalizedAngle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class RobotConstants
{
    @Configurable
    public static class General
    {
        public static ConstantsPresets.Preset PRESET_OPTION = ConstantsPresets.Preset.TESTING;
        public static boolean REGRESSION_TESTING_MODE = false;
        public static double MOTOR_ACCELERATION_FILTER_FACTOR = 0.5;
    }

    @Configurable
    public static class Autonomous
    {
        public static int WAIT_SYS_READY_MS = 4000;
        public static int SHOOT_LENGTH_MS = 1500;
        public static boolean SAVE_END_AUTONOMOUS_POSE = true;
        public static long AUTONOMOUS_POSE_TIMEOUT_MS = 90_000; // use this pose if it was created in the last 90,000 milliseconds (90 seconds)
        public static String AUTONOMOUS_POSE_FILE_NAME = "saved_pose.txt";
        public static boolean DELETE_AUTONOMOUS_POSE_AFTER_FIRST_READ = false;
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
        public static int[] GOAL_IDS = new int[]{20, 24};
        public static int[] OBELISK_IDS = new int[]{21, 22, 23};
    }

    @Configurable
    // Hardware Names
    public static class Drive
    {
        public static DriveMode DRIVE_MODE = DriveMode.RAW_ROBOT_CENTRIC;

        @IgnoreConfigurable
        public static class WHEEL_NAMES
        {
            public static String FRONT_LEFT = "frontLeft";
            public static String FRONT_RIGHT = "frontRight";
            public static String BACK_LEFT = "backLeft";
            public static String BACK_RIGHT = "backRight";
        }

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
        @Sorter(sort = 1)
        public static double INPUT_GEAR_RATIO = 1;
        public static double OUTPUT_GEAR_RATIO = 1;

        @Sorter(sort = 2)
        public static int BASE_RPM = 3000;

        @Sorter(sort = 3)
        public static boolean ON_BY_DEFAULT = false;
        @Sorter(sort = 4)
        public static boolean AUTO_DISTANCE_ADJUSMENT = true;

        public static boolean USE_LUT = false;

        public static Distance OFFSET_FROM_CENTER = new Distance(-4.5, DistanceUnit.INCH);

        public static Distance OFFSET_FROM_TURRET = new Distance(4, DistanceUnit.INCH);

        @IgnoreConfigurable
        public static class Name
        {
            public static String TOP = "topOuttake";
            public static String BOTTOM = "bottomOuttake";
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
            public static int RPM = 60;
            @Sorter(sort = 1)
            public static int RPM_ACCELERATION = 250;
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
        public static boolean USE_LINEAR_TOLERANCE_RADIUS = false;
        @Sorter(sort = 4)
        public static Angle FORWARD_ANGLE = new Angle(0, AngleUnit.DEGREES);

        public static boolean TARGET_ONLY_WHEN_INTENDING_TO_SHOOT = true;

        /**
         * Calibration offset to compensate for systematic aiming error.
         * Positive = turret aims more to the LEFT (CCW when viewed from above)
         * Negative = turret aims more to the RIGHT (CW when viewed from above)
         *
         * If balls consistently miss to the LEFT of the goal, use a NEGATIVE value.
         * If balls consistently miss to the RIGHT of the goal, use a POSITIVE value.
         */
        @Sorter(sort = 5)
        public static Angle AIM_CALIBRATION_OFFSET = new Angle(0, AngleUnit.DEGREES);
        @Sorter(sort = 6)
        public static UnnormalizedAngle[] TURN_LIMITS = new UnnormalizedAngle[]
        {
                new UnnormalizedAngle(-70, UnnormalizedAngleUnit.DEGREES), // negative clockwise, positive counter clockwise
                new UnnormalizedAngle(70, UnnormalizedAngleUnit.DEGREES) }; // in both directions, so if 0 is forward
        @Sorter(sort = 7)
        public static boolean AUTO_AIM_TO_GOAL = false;

        @Sorter(sort = 8)
        // only change p and d
        public static PIDFCoefficients PIDF = new PIDFCoefficients(0.08, 0.00, 0.0007, 0.00);

        @Sorter(sort = 11)
        public static boolean ROTATION_COMPENSATION_ENABLED = true;
        @Sorter(sort = 12)
        public static double ROTATION_COMPENSATION_FF = 0.0008; // Feedforward gain: power per deg/s of robot rotation

        @Sorter(sort = 13)
        public static Angle SAFETY_MARGIN = new Angle(10, AngleUnit.DEGREES);

        // right hand rule: forward = x, strafe = y
        public static Distance OFFSET_FROM_CENTER = new Distance(-4.5, DistanceUnit.INCH);
    }

    @Configurable
    public static class Transfer
    {
        @Sorter(sort = 0)
        public static String NAME = "transfer";

        @Sorter(sort = 1)
        public static double OPEN_ANGLE = 95; // Open means the transfer is allowing balls to pass through

        @Sorter(sort = 2)
        public static double CLOSE_INTAKE_ANGLE = 0; // 210; // An angle where the trapdoor blocks balls from entering

        public static long WAIT_BEFORE_TRANSFER = 500;
    }

    @Configurable
    public static class Intake
    {
        @Sorter(sort = 0)
        public static String NAME = "intake";

        @Sorter(sort = 1)
        public static double intakePower = 1.0;
        @Sorter(sort = 2)
        public static double outtakePower = 1.0;
        @Sorter(sort = 3)
        public static double minimumTransferPower = 0.72;
        public static double maximumTransferPower = 0.9;

        @Sorter(sort = 7)
        public static boolean INTAKE_BY_DEFAULT = false;

        public static PIDFCoefficients PIDF = new PIDFCoefficients(0.001, 0.1, 0, 0.0006);
    }

    @Configurable
    public static class Odometry
    {
        @Sorter(sort = 0)
        public static Pose2d DEFAULT_POSE = new Pose2d(
                new FieldCoordinate(new Distance(72, DistanceUnit.INCH), new Distance(72, DistanceUnit.INCH),
                        CoordinateSystem.DECODE_PEDROPATH),
                new FieldHeading(new Angle(90, AngleUnit.DEGREES), CoordinateSystem.DECODE_PEDROPATH));

        public static boolean SET_FORWARD_DIRECTION_BASED_ON_TEAM = true;
        public static boolean ENABLE_CORNER_LOCALIZATION = true;
        public static boolean RESET_PINPOINT_FULLY = true;
        public static boolean RESET_PINPOINT = true;

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
                public static double LENS_FX = 947.111; // 958.876
                public static double LENS_FY = 947.111; // 958.876
                public static double LENS_CX = 635.055; // 654.11
                public static double LENS_CY = 365.043; // 358.336
            }

            @Configurable
            public static class Offset
            {
                // Position of camera relative to robot center
                // Uses FTC SDK conventions for setCameraPose():
                // X: Left/right (positive is RIGHT, negative is LEFT)
                // Y: Forward/backward (positive is FORWARD)
                // Z: Positive is UP
                @Sorter(sort = 0)
                public static Distance X = new Distance(0.25, DistanceUnit.INCH); // camera left/right of center (positive = right)
                @Sorter(sort = 1)
                public static Distance Y = new Distance(5.5, DistanceUnit.INCH); // camera forward of center
                @Sorter(sort = 2)
                public static Distance Z = new Distance(9.5, DistanceUnit.INCH);

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

            @Sorter(sort = 1)
            public static Distance WHEEL_DIAMETER = new Distance(35, DistanceUnit.MM);

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