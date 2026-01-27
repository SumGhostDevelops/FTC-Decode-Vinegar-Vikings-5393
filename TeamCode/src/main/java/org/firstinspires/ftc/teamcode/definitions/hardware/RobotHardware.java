package org.firstinspires.ftc.teamcode.definitions.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.util.motors.MotorXP;

import java.util.List;

public class RobotHardware
{
    public MotorXP frontLeft, frontRight, backLeft, backRight, intake, turret, outtake;
    public Motor.Encoder dwFwd, dwStrf;
    public ServoEx transfer;
    public IMU imu;
    public List<LynxModule> hubs;
    public Pinpoint pinpoint;
    public WebcamName webcam;
    public VoltageSensor battery;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        try
        {
            // --- Drive Motors ---
            // Split initialization so a missing/failed motor doesn't prevent others from initializing.
            boolean driveOk = true;

            try
            {
                frontLeft = new MotorXP(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_LEFT, Motor.GoBILDA.RPM_312);
                frontLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            catch (Exception e)
            {
                driveOk = false;
                frontLeft = null;
            }

            try
            {
                frontRight = new MotorXP(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.FRONT_RIGHT, Motor.GoBILDA.RPM_312);
                frontRight.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            catch (Exception e)
            {
                driveOk = false;
                frontRight = null;
            }

            try
            {
                backLeft = new MotorXP(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_LEFT, Motor.GoBILDA.RPM_312);
                backLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            catch (Exception e)
            {
                driveOk = false;
                backLeft = null;
            }

            try
            {
                backRight = new MotorXP(hardwareMap, RobotConstants.Drive.WHEEL_NAMES.BACK_RIGHT, Motor.GoBILDA.RPM_312);
                backRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            catch (Exception e)
            {
                driveOk = false;
                backRight = null;
            }

            if (driveOk)
            {
                MotorGroup driveGroup = getDriveMotorExPlusGroup();
                driveGroup.setRunMode(Motor.RunMode.RawPower);
                driveGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            }
            else
            {
                telemetry.log().add("Warning: One or more drive motors not found");
            }
        }
        catch (Exception e)
        {
            // This outer catch is retained as a safeguard; specific failures are handled above.
            telemetry.log().add("Warning: One or more drive motors not found");
        }

        // --- Mechanisms ---
        boolean outtakeOk = true;
        try
        {
            outtake = new MotorXP(hardwareMap, RobotConstants.Outtake.Name.LAUNCHER_LEFT, Motor.GoBILDA.BARE);

            outtake.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double[] veloCoeffs = RobotConstants.Outtake.Coefficients.veloCoeffs;
            outtake.setVelocityPIDFCoefficients(veloCoeffs[0], veloCoeffs[1], veloCoeffs[2], veloCoeffs[3]);

            outtake.setRpmTolerance(RobotConstants.Outtake.Tolerance.RPM);
            outtake.setAccelTolerance(RobotConstants.Outtake.Tolerance.RPM_ACCELERATION);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Left outtake motor not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            turret = new MotorXP(hardwareMap, RobotConstants.Turret.NAME, Motor.GoBILDA.RPM_435);
            turret.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setRunMode(Motor.RunMode.PositionControl);
            double[] pidf = RobotConstants.Turret.pidf;
            turret.motorEx.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
            //turret.motorEx.setPositionPIDFCoefficients(pidf[0]);
            turret.setTargetPosition(0);
            turret.setPositionTolerance(RobotConstants.Turret.TOLERANCE); // Allow 5 ticks tolerance for atTargetPosition()
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            transfer = new ServoEx(hardwareMap, RobotConstants.Transfer.NAME, 0, 360);
            transfer.set(RobotConstants.Transfer.CLOSED_INTAKE_ANGLE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Transfer servo not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            intake = new MotorXP(hardwareMap, RobotConstants.Intake.NAME, Motor.GoBILDA.RPM_1620);
            intake.setRunMode(Motor.RunMode.VelocityControl);
            intake.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Intake motor not found");
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
            imu = hardwareMap.get(IMU.class, RobotConstants.Odometry.IMU.NAME);

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ));

            imu.initialize(parameters);
            imu.resetYaw();

            hubs = hardwareMap.getAll(LynxModule.class);

            hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Control Hub IMU not found");
        }

        try
        {
            dwFwd = backLeft.encoder;
            dwFwd.setDirection(Motor.Direction.REVERSE);
            dwStrf = backRight.encoder;
            dwStrf.setDirection(Motor.Direction.REVERSE);
            dwFwd.reset();
            dwStrf.reset();
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Forward and/or strafe encoders not found");
        }

        try
        {
            webcam = hardwareMap.get(WebcamName.class, RobotConstants.Odometry.Webcam.NAME);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Webcam not found");
        }

        try
        {
            battery = hardwareMap.voltageSensor.iterator().next();
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Voltage sensor not found");
        }
    }

    public MotorGroup getDriveMotorExPlusGroup()
    {
        return new MotorGroup(frontLeft, frontRight, backLeft, backRight);
    }

    public MotorXP[] getDriveArray()
    {
        return new MotorXP[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void clearHubCache()
    {
        hubs.forEach(LynxModule::clearBulkCache);
    }
}