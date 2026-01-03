package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

public class RobotHardware
{
    public MotorExPlus frontLeft, frontRight, backLeft, backRight, intake, turret, outtakeLeft, outtakeRight;
    public Motor.Encoder dwFwd, dwStrf;
    public ServoEx transfer;
    public IMU imu;
    public Pinpoint pinpoint;
    public WebcamName webcam;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        try
        {
            // --- Drive Motors ---
            // Split initialization so a missing/failed motor doesn't prevent others from initializing.
            boolean driveOk = true;

            try
            {
                frontLeft = new MotorExPlus(hardwareMap, RobotConstants.Drive.FRONT_LEFT, Motor.GoBILDA.RPM_312);
                frontLeft.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            catch (Exception e)
            {
                driveOk = false;
                frontLeft = null;
            }

            try
            {
                frontRight = new MotorExPlus(hardwareMap, RobotConstants.Drive.FRONT_RIGHT, Motor.GoBILDA.RPM_312);
                frontRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            catch (Exception e)
            {
                driveOk = false;
                frontRight = null;
            }

            try
            {
                backLeft = new MotorExPlus(hardwareMap, RobotConstants.Drive.BACK_LEFT, Motor.GoBILDA.RPM_312);
                backLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            catch (Exception e)
            {
                driveOk = false;
                backLeft = null;
            }

            try
            {
                backRight = new MotorExPlus(hardwareMap, RobotConstants.Drive.BACK_RIGHT, Motor.GoBILDA.RPM_312);
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
            outtakeLeft = new MotorExPlus(hardwareMap, RobotConstants.Outtake.Name.LAUNCHER_LEFT, Motor.GoBILDA.BARE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Left outtake motor not found");
            outtakeOk = false;
        }

        try
        {
            outtakeRight = new MotorExPlus(hardwareMap, RobotConstants.Outtake.Name.LAUNCHER_RIGHT, Motor.GoBILDA.BARE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Right outtake motor not found");
            outtakeOk = false;
        }

        if (outtakeOk)
        {
            MotorExPlusGroup outtakeGroup = getOuttakeMotorExPlusGroup();
            outtakeGroup.setRunMode(Motor.RunMode.VelocityControl);
            outtakeGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            double[] veloCoeffs = RobotConstants.Outtake.Coefficients.veloCoeffs;
            double[] ffCoeffs = RobotConstants.Outtake.Coefficients.ffCoeffs;

            outtakeGroup.setVeloCoefficients(veloCoeffs[0], veloCoeffs[1], veloCoeffs[2]);
            outtakeGroup.setFeedforwardCoefficients(ffCoeffs[0], ffCoeffs[1], ffCoeffs[2]);
        }

        try
        {
            turret = new MotorExPlus(hardwareMap, RobotConstants.Turret.TURRET, Motor.GoBILDA.RPM_312);
            turret.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setRunMode(Motor.RunMode.PositionControl);
            turret.setPositionTolerance(10); // Allow 10 ticks tolerance for atTargetPosition()
            //turret.setPositionCoefficient(RobotConstants.Turret.posCoeff);

            //double[] ffCoeffs = RobotConstants.Turret.ffCoeffs;
            //turret.setFeedforwardCoefficients(ffCoeffs[0], ffCoeffs[1], ffCoeffs[2]);
            //turret.setPositionTolerance(RobotConstants.Turret.TOLERANCE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            transfer = new ServoEx(hardwareMap, RobotConstants.Transfer.TRANSFER, RobotConstants.Transfer.SERVO_RANGE, AngleUnit.DEGREES);
            //Servo transfer = hardwareMap.get(Servo.class, RobotConstants.Transfer.TRANSFER);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Transfer servo not found");
            telemetry.log().add(e.getMessage());
        }

        try
        {
            intake = new MotorExPlus(hardwareMap, RobotConstants.Intake.NAME, Motor.GoBILDA.RPM_1620);
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Intake motor not found");
        }

        // Odometry
        try
        {
            pinpoint = hardwareMap.get(Pinpoint.class, RobotConstants.Odometry.Pinpoint.NAME);

            pinpoint.setOffsets(RobotConstants.Odometry.Deadwheels.Forward.OFFSET.magnitude, RobotConstants.Odometry.Deadwheels.Strafe.OFFSET.magnitude, RobotConstants.Odometry.Deadwheels.Forward.OFFSET.unit);

            double counts_per_unit = (double) RobotConstants.Odometry.Deadwheels.COUNTS_PER_REVOLUTION / RobotConstants.Odometry.Deadwheels.WHEEL_CIRCUMFERENCE.magnitude;
            pinpoint.setEncoderResolution(counts_per_unit, RobotConstants.Odometry.Deadwheels.WHEEL_CIRCUMFERENCE.unit);

            pinpoint.setEncoderDirections(Pinpoint.EncoderDirection.FORWARD, Pinpoint.EncoderDirection.FORWARD);

            pinpoint.resetPosAndIMU();
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: goBilda Pinpoint not found");
        }

        try
        {
            imu = hardwareMap.get(IMU.class, RobotConstants.Odometry.IMU.NAME);
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ));

            imu.initialize(parameters);
            imu.resetYaw();
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Control Hub IMU not found");
        }

        try
        {
            dwFwd = backRight.encoder;
            dwStrf = backLeft.encoder;
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
    }

    public MotorGroup getDriveMotorExPlusGroup()
    {
        return new MotorGroup(frontLeft, frontRight, backLeft, backRight);
    }

    public MotorExPlus[] getDriveArray()
    {
        return new MotorExPlus[]{frontLeft, frontRight, backLeft, backRight};
    }

    public MotorExPlusGroup getOuttakeMotorExPlusGroup()
    {
        return new MotorExPlusGroup(outtakeLeft, outtakeRight);
    }

    public MotorExPlus[] getOuttakeArray()
    {
        return new MotorExPlus[]{outtakeLeft, outtakeRight};
    }
}