package org.firstinspires.ftc.teamcode.definitions.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    public VoltageSensor battery;
    private double cachedVoltage = 12.0;

    public List<LynxModule> hubs;

    public Pinpoint pinpoint;
    public WebcamName webcam;

    public PowerMotor frontLeft, frontRight, backLeft, backRight;
    public PowerMotor intake;
    public VelocityMotorGroup outtake;
    public PositionMotor turret;
    public ServoEx transfer;

    private String pinpointName = RobotConstants.Odometry.Pinpoint.NAME;
    private Distance forwardWheelOffset = RobotConstants.Odometry.Deadwheels.Forward.OFFSET;
    private Distance strafeWheelOffset = RobotConstants.Odometry.Deadwheels.Strafe.OFFSET;
    private String webcamName = RobotConstants.Odometry.Webcam.NAME;

    private String frontLeftName = RobotConstants.Drive.WHEEL_NAMES.FRONT_LEFT;
    private String frontRightName = RobotConstants.Drive.WHEEL_NAMES.FRONT_RIGHT;
    private String backLeftName = RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT;
    private String backRightName = RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT;

    private String outtakeLauncherLeftName = RobotConstants.Outtake.Name.TOP;
    private String outtakeLauncherRightName = RobotConstants.Outtake.Name.BOTTOM;
    private PIDFCoefficients outtakePIDF = RobotConstants.Outtake.Coefficients.PIDF;
    private int outtakeToleranceRPM = RobotConstants.Outtake.Tolerance.RPM;
    private int outtakeToleranceRPMAccel = RobotConstants.Outtake.Tolerance.RPM_ACCELERATION;
    private double outtakeInputGearRatio = RobotConstants.Outtake.INPUT_GEAR_RATIO;
    private double outtakeOutputGearRatio = RobotConstants.Outtake.OUTPUT_GEAR_RATIO;

    private String turretName = RobotConstants.Turret.NAME;
    private double turretGearRatio = RobotConstants.Turret.GEAR_RATIO;
    private PIDFCoefficients turretPIDF = RobotConstants.Turret.PIDF;
    private double turretToleranceDegrees = RobotConstants.Turret.TOLERANCE.getDegrees();

    private String transferName = RobotConstants.Transfer.NAME;
    private double transferCloseIntakeAngle = RobotConstants.Transfer.CLOSE_INTAKE_ANGLE;

    private String intakeName = RobotConstants.Intake.NAME;
    private PIDFCoefficients intakePIDF = RobotConstants.Intake.PIDF;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        try
        {
            hubs = hardwareMap.getAll(LynxModule.class);

            hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Hubs not found");
        }

        try
        {
            battery = hardwareMap.voltageSensor.iterator().next();
            cachedVoltage = battery.getVoltage();
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Voltage sensor not found");
        }

        // Odometry
        try
        {
            pinpoint = hardwareMap.get(Pinpoint.class, pinpointName);

            // Pinpoint convention: X offset (left=positive), Y offset (forward=positive)
            // Our RobotConstants convention matches Pinpoint: left=positive,
            // forward=positive
            DistanceUnit dUnit = forwardWheelOffset.unit;

            pinpoint.setOffsets(
                    forwardWheelOffset.magnitude, // Left is positive in both conventions
                    strafeWheelOffset.toUnit(dUnit).magnitude, // Forward is positive in both conventions
                    dUnit);

            pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(Pinpoint.EncoderDirection.FORWARD, Pinpoint.EncoderDirection.REVERSED);

            pinpoint.recalibrateIMU();
        } catch (Exception e)
        {
            telemetry.log().add("Warning: goBilda Pinpoint not found");
        }

        try
        {
            webcam = hardwareMap.get(WebcamName.class, webcamName);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Webcam not found");
        }

        // Drive
        try
        {
            frontLeft = new PowerMotor(new MotorEx(hardwareMap, frontLeftName, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            frontRight = new PowerMotor(new MotorEx(hardwareMap, frontRightName, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            backLeft = new PowerMotor(new MotorEx(hardwareMap, backLeftName, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            backRight = new PowerMotor(new MotorEx(hardwareMap, backRightName, Motor.GoBILDA.RPM_312), () -> cachedVoltage)
                    .setMotorDirection(Motor.Direction.FORWARD)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: One or more drive motors not found");
        }

        // Outtake
        try
        {
            MotorEx outtakeTopEx = new MotorEx(hardwareMap, outtakeLauncherLeftName, Motor.GoBILDA.BARE);
            MotorEx outtakeBottomEx = new MotorEx(hardwareMap, outtakeLauncherRightName, Motor.GoBILDA.BARE);

            outtakeBottomEx.encoder = outtakeTopEx.encoder;

            VelocityMotor outtakeTop = new VelocityMotor(outtakeTopEx, () -> cachedVoltage)
                    .setDistancePerPulse(outtakeInputGearRatio, outtakeOutputGearRatio);
            VelocityMotor outtakeBottom = new VelocityMotor(outtakeBottomEx, () -> cachedVoltage)
                    .setDistancePerPulse(outtakeInputGearRatio, outtakeOutputGearRatio)
                    .setMotorDirection(Motor.Direction.REVERSE);

            outtake = new VelocityMotorGroup(outtakeTop, outtakeBottom)
                    .setVoltageCompensation(12)
                    .setControllerType(VelocityMotor.VelocityController.TakeBackHalf)
                    .setPIDF(outtakePIDF)
                    .setTolerance(outtakeToleranceRPM, outtakeToleranceRPMAccel);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: One or more outtake motors not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            turret = new PositionMotor(new MotorEx(hardwareMap, turretName, Motor.GoBILDA.RPM_435), () -> cachedVoltage)
                    .setVoltageCompensation(12) // 100% power >= 12 volts
                    .usePower(1)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                    .setDistancePerPulse(1.0, turretGearRatio, AngleUnit.DEGREES) // keep in degrees
                    .setControllerType(PositionMotor.PositionController.SquIDF)
                    .setPIDF(turretPIDF)
                    .setPositionTolerance(turretToleranceDegrees);

            turret.setTargetDistance(0);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            transfer = new ServoEx(hardwareMap, transferName, 0, 360);
            transfer.set(transferCloseIntakeAngle);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Transfer servo not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            intake = new PowerMotor(new MotorEx(hardwareMap, intakeName, Motor.GoBILDA.RPM_1620), () -> cachedVoltage)
                    .setVoltageCompensation(12)
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        } catch (Exception e)
        {
            telemetry.log().add("Warning: Intake motor not found");
        }
    }

    public void clearHubCache()
    {
        hubs.forEach(LynxModule::clearBulkCache);
    }

    public void readBattery()
    {
        if (battery != null)
        {
            cachedVoltage = battery.getVoltage();
        }
    }
}