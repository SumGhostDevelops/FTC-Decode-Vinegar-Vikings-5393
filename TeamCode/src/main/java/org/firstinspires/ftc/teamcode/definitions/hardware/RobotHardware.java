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
import org.firstinspires.ftc.teamcode.util.motors.PositionMotor;
import org.firstinspires.ftc.teamcode.util.motors.PowerMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotorGroup;

import java.util.List;

public class RobotHardware
{
    public VoltageSensor battery;
    public List<LynxModule> hubs;

    public Pinpoint pinpoint;
    public WebcamName webcam;

    public PowerMotor frontLeft, frontRight, backLeft, backRight;
    public PowerMotor intake;
    public VelocityMotorGroup outtake;
    public PositionMotor turret;
    public ServoEx transfer;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
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
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Voltage sensor not found");
        }

        // Odometry
        try
        {
            pinpoint = hardwareMap.get(Pinpoint.class, RobotConstants.Odometry.Pinpoint.NAME);

            // Pinpoint convention: X offset (left=positive), Y offset (forward=positive)
            // Our RobotConstants convention matches Pinpoint: left=positive, forward=positive
            DistanceUnit dUnit = RobotConstants.Odometry.Deadwheels.Forward.OFFSET.unit;

            pinpoint.setOffsets(
                    RobotConstants.Odometry.Deadwheels.Forward.OFFSET.magnitude,   // Left is positive in both conventions
                    RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.toUnit(dUnit).magnitude,    // Forward is positive in both conventions
                    dUnit
            );

            pinpoint.setEncoderResolution(Pinpoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(Pinpoint.EncoderDirection.FORWARD, Pinpoint.EncoderDirection.REVERSED);

            pinpoint.recalibrateIMU();
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: goBilda Pinpoint not found");
        }

        try
        {
            webcam = hardwareMap.get(WebcamName.class, RobotConstants.Odometry.Webcam.NAME);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Webcam not found");
        }

        // Drive
        try
        {
            frontLeft = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_LEFT, Motor.GoBILDA.RPM_312))
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            frontRight = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_RIGHT, Motor.GoBILDA.RPM_312))
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            backLeft = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT, Motor.GoBILDA.RPM_312))
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            backRight = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT, Motor.GoBILDA.RPM_312))
                    .setMotorDirection(Motor.Direction.FORWARD)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: One or more drive motors not found");
        }

        // Outtake
        try
        {
            VelocityMotor outtakeLeft = new VelocityMotor(new MotorEx(hardwareMap, RobotConstants.Outtake.Name.LAUNCHER_LEFT, Motor.GoBILDA.BARE), battery);
            // VelocityMotor outtakeRight = new MotorEx
            outtake = new VelocityMotorGroup(outtakeLeft)
                    .setVoltageCompensation(12)
                    .setControllerType(VelocityMotor.VelocityController.TakeBackHalf)
                    .setPIDF(RobotConstants.Outtake.Coefficients.PIDF)
                    .setTolerance(RobotConstants.Outtake.Tolerance.RPM, RobotConstants.Outtake.Tolerance.RPM_ACCELERATION);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: One or more outtake motors not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            turret = new PositionMotor(new MotorEx(hardwareMap, RobotConstants.Turret.NAME, Motor.GoBILDA.RPM_435), battery)
                    .setVoltageCompensation(12) // 100% power >= 12 volts
                    .usePower(1)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                    .setDistancePerPulse(1.0, RobotConstants.Turret.GEAR_RATIO, AngleUnit.DEGREES) // keep in degrees
                    .setControllerType(PositionMotor.PositionController.SquIDF)
                    .setPIDF(RobotConstants.Turret.PIDF)
                    .setPositionTolerance(RobotConstants.Turret.TOLERANCE);

            turret.setTargetDistance(0);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            transfer = new ServoEx(hardwareMap, RobotConstants.Transfer.NAME, 0, 360);
            transfer.set(RobotConstants.Transfer.CLOSE_INTAKE_ANGLE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Transfer servo not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            intake = new PowerMotor(new MotorEx(hardwareMap, RobotConstants.Intake.NAME, Motor.GoBILDA.RPM_1620))
                    .setVoltageCompensation(12)
                    .setMotorDirection(Motor.Direction.REVERSE)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Intake motor not found");
        }
    }

    public void clearHubCache()
    {
        hubs.forEach(LynxModule::clearBulkCache);
    }
}