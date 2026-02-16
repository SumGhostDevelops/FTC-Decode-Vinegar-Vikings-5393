package org.firstinspires.ftc.teamcode.definitions.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.motors.PositionMotor;
import org.firstinspires.ftc.teamcode.util.motors.PowerMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotorGroup;

import java.util.List;

public class RobotHardware
{
    // Constants for internal configuration
    private final String pinpointName = RobotConstants.Odometry.Pinpoint.NAME;
    private final Distance forwardWheelOffset = RobotConstants.Odometry.Deadwheels.Forward.OFFSET;
    private final Distance strafeWheelOffset = RobotConstants.Odometry.Deadwheels.Strafe.OFFSET;
    // Global Hardware
    public VoltageSensor battery;
    public List<LynxModule> hubs;
    // Subsystems (Nullable if not built)
    public Pinpoint pinpoint;
    public WebcamName webcam;
    public PowerMotor frontLeft, frontRight, backLeft, backRight;
    public PowerMotor intake;
    public VelocityMotorGroup outtake;
    public PositionMotor turret;
    public ServoEx transfer;
    private double cachedVoltage = 12.0;
    // Pinpoint State
    private boolean pinpointConfigured = false;
    private boolean pinpointCalibrationStarted = false;
    private boolean pinpointReady = false;

    /**
     * Private constructor used by the Builder.
     */
    private RobotHardware(Builder builder)
    {
        this.hubs = builder.hubs;
        this.battery = builder.battery;
        this.cachedVoltage = builder.cachedVoltage;

        this.frontLeft = builder.frontLeft;
        this.frontRight = builder.frontRight;
        this.backLeft = builder.backLeft;
        this.backRight = builder.backRight;

        this.outtake = builder.outtake;
        this.turret = builder.turret;
        this.intake = builder.intake;
        this.transfer = builder.transfer;

        this.pinpoint = builder.pinpoint;
        this.webcam = builder.webcam;
    }

    public void clearHubCache()
    {
        if (hubs != null)
        {
            hubs.forEach(LynxModule::clearBulkCache);
        }
    }

    public void readBattery()
    {
        if (battery != null)
        {
            cachedVoltage = battery.getVoltage();
        }
    }

    public double getCachedVoltage()
    {
        return cachedVoltage;
    }

    /**
     * Attempts to configure and calibrate the Pinpoint. Call this repeatedly during init
     * until isPinpointReady() returns true.
     *
     * @return true if Pinpoint is fully ready to use
     */
    public boolean loadPinpoint(HardwareMap hardwareMap, Telemetry telemetry)
    {
        if (pinpointReady) return true;

        // Attempt to fetch if missing (redundant check if built correctly, but safe)
        if (pinpoint == null)
        {
            try
            {
                pinpoint = hardwareMap.get(Pinpoint.class, pinpointName);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: goBilda Pinpoint not found");
                return false;
            }
        }

        pinpoint.update();
        Pinpoint.DeviceStatus status = pinpoint.getDeviceStatus();

        if (status != Pinpoint.DeviceStatus.READY) return false;

        // Step 1: Configure
        if (!pinpointConfigured)
        {
            try
            {
                DistanceUnit dUnit = forwardWheelOffset.unit;
                pinpoint.setOffsets(forwardWheelOffset.magnitude, strafeWheelOffset.toUnit(dUnit).magnitude, dUnit);
                pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(Pinpoint.EncoderDirection.FORWARD, Pinpoint.EncoderDirection.REVERSED);
                pinpointConfigured = true;
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Failed to configure Pinpoint");
                return false;
            }
        }

        // Step 2: Recalibrate IMU
        if (!pinpointCalibrationStarted)
        {
            pinpoint.recalibrateIMU();
            pinpointCalibrationStarted = true;
            return false;
        }

        // Step 3: Ready
        pinpointReady = true;
        return true;
    }

    // --- Runtime Configuration Methods (Pinpoint Logic) ---

    public boolean isPinpointReady()
    {
        return pinpointReady;
    }

    /**
     * Builder Class for RobotHardware
     */
    public static class Builder
    {
        private final HardwareMap hardwareMap;
        private final Telemetry telemetry;

        // Base
        private List<LynxModule> hubs;
        private VoltageSensor battery;
        private double cachedVoltage = 12.0;

        // Subsystems
        private PowerMotor frontLeft, frontRight, backLeft, backRight;
        private VelocityMotorGroup outtake;
        private PositionMotor turret;
        private PowerMotor intake;
        private ServoEx transfer;
        private Pinpoint pinpoint;
        private WebcamName webcam;

        public Builder(HardwareMap hardwareMap, Telemetry telemetry)
        {
            this.hardwareMap = hardwareMap;
            this.telemetry = telemetry;
            initializeBaseHardware();
        }

        private void initializeBaseHardware()
        {
            try
            {
                hubs = hardwareMap.getAll(LynxModule.class);
                hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Hubs not found");
            }

            try
            {
                battery = hardwareMap.voltageSensor.iterator().next();
                cachedVoltage = battery.getVoltage();
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Voltage sensor not found");
            }
        }

        public Builder withAll()
        {
            return this
                    .withDrive()
                    .withOuttake()
                    .withTurret()
                    .withIntake()
                    .withTransfer()
                    .withOdometry();
        }

        public Builder withDrive()
        {
            try
            {
                frontLeft = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_LEFT, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                        .setMotorDirection(Motor.Direction.REVERSE)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

                frontRight = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_RIGHT, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                        .setMotorDirection(Motor.Direction.REVERSE)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

                backLeft = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                        .setMotorDirection(Motor.Direction.REVERSE)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

                backRight = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                        .setMotorDirection(Motor.Direction.FORWARD)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Drive motors failed to initialize");
            }
            return this;
        }

        public Builder withOuttake()
        {
            try
            {
                MotorEx outtakeTopEx = new MotorEx(hardwareMap, RobotConstants.Outtake.Name.TOP, Motor.GoBILDA.BARE);
                MotorEx outtakeBottomEx = new MotorEx(hardwareMap, RobotConstants.Outtake.Name.BOTTOM, Motor.GoBILDA.BARE);

                outtakeBottomEx.encoder = outtakeTopEx.encoder;

                double inputRatio = RobotConstants.Outtake.INPUT_GEAR_RATIO;
                double outputRatio = RobotConstants.Outtake.OUTPUT_GEAR_RATIO;

                VelocityMotor outtakeTop = new VelocityMotor(outtakeTopEx, () -> cachedVoltage)
                        .setDistancePerPulse(inputRatio, outputRatio);
                VelocityMotor outtakeBottom = new VelocityMotor(outtakeBottomEx, () -> cachedVoltage)
                        .setDistancePerPulse(inputRatio, outputRatio)
                        .setMotorDirection(Motor.Direction.REVERSE);

                outtake = new VelocityMotorGroup(outtakeTop, outtakeBottom)
                        .setVoltageCompensation(12)
                        .setControllerType(VelocityMotor.VelocityController.TakeBackHalf)
                        .setPIDF(RobotConstants.Outtake.Coefficients.PIDF)
                        .setTolerance(RobotConstants.Outtake.Tolerance.RPM, RobotConstants.Outtake.Tolerance.RPM_ACCELERATION);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Outtake failed to initialize: " + e.getMessage());
            }
            return this;
        }

        public Builder withTurret()
        {
            try
            {
                turret = new PositionMotor(new MotorEx(hardwareMap, RobotConstants.Turret.NAME, Motor.GoBILDA.RPM_435), () -> cachedVoltage)
                        .setVoltageCompensation(12)
                        .usePower(1)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                        .setDistancePerPulse(1.0, RobotConstants.Turret.GEAR_RATIO, AngleUnit.DEGREES)
                        .setControllerType(PositionMotor.PositionController.SquIDF)
                        .setPIDF(RobotConstants.Turret.PIDF)
                        .setPositionTolerance(RobotConstants.Turret.TOLERANCE.getDegrees());

                turret.setTargetDistance(0);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Turret failed to initialize: " + e.getMessage());
            }
            return this;
        }

        public Builder withIntake()
        {
            try
            {
                intake = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Intake.NAME, Motor.GoBILDA.RPM_1620), () -> cachedVoltage)
                        .setVoltageCompensation(12)
                        .setMotorDirection(Motor.Direction.REVERSE)
                        .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Intake failed to initialize");
            }
            return this;
        }

        public Builder withTransfer()
        {
            try
            {
                transfer = new ServoEx(hardwareMap, RobotConstants.Transfer.NAME, 0, 360);
                transfer.set(RobotConstants.Transfer.CLOSE_INTAKE_ANGLE);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Transfer servo failed to initialize");
            }
            return this;
        }

        public Builder withOdometry()
        {
            // Pinpoint
            try
            {
                pinpoint = hardwareMap.get(Pinpoint.class, RobotConstants.Odometry.Pinpoint.NAME);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: goBilda Pinpoint not found");
            }
            // Webcam
            try
            {
                webcam = hardwareMap.get(WebcamName.class, RobotConstants.Odometry.Webcam.NAME);
            }
            catch (Exception e)
            {
                telemetry.log().add("Warning: Webcam not found");
            }
            return this;
        }

        public RobotHardware build()
        {
            return new RobotHardware(this);
        }
    }
}