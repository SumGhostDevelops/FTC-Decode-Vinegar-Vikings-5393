package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.GoBildaPinpointDriver;

public class RobotHardware
{
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx outtakeLeftMotor, outtakeRightMotor, turretMotor, intakeMotor;
    public CRServo transferServo;
    public GoBildaPinpointDriver pinpoint;
    public WebcamName webcam;

    // Dead wheel encoders (often accessed via specific motor ports)
    public DcMotorEx xEncoder, yEncoder;

    public Telemetry telemetry;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        // --- Drive Motors ---
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_RIGHT);

        // Directions (Adjust to ensure all wheels spin forward when commanded positive)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, frontLeft, frontRight, backLeft, backRight);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, frontLeft, frontRight, backLeft, backRight);

        // --- Mechanisms ---
        try
        {
            outtakeLeftMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.LAUNCHER_LEFT);
            outtakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outtakeLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // For Velocity Control
            outtakeLeftMotor.setVelocityPIDFCoefficients(10, 4, 5, -4.5);

            outtakeRightMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.LAUNCHER_RIGHT);
            outtakeRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outtakeRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // For Velocity Control
            outtakeRightMotor.setVelocityPIDFCoefficients(10, 4, 5, -4.5);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Outtake motors not found");
        }

        try
        {
            turretMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.TURRET);
            outtakeRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found");
        }

        try
        {
            transferServo = hardwareMap.get(CRServo.class, RobotConstants.Transfer.TRANSFER);
            transferServo.setDirection(CRServo.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Transfer servo not found");
        }

        try
        {
            intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.INTAKE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Intake motor not found");
        }

        // Odometry
        try
        {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.Odometry.PINPOINT);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: goBilda Pinpoint not found");
        }

        try
        {
            webcam = hardwareMap.get(WebcamName.class, RobotConstants.Odometry.WEBCAM);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Webcam not found");
        }

        try
        {
            xEncoder = hardwareMap.get(DcMotorEx.class, RobotConstants.Odometry.Deadwheels.PAR_X);
            yEncoder = hardwareMap.get(DcMotorEx.class, RobotConstants.Odometry.Deadwheels.PERP_Y);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Dead wheel encoders not found");
        }
    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors)
    {
        for (DcMotor motor : motors) motor.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior, DcMotor... motors)
    {
        for (DcMotor motor : motors) motor.setZeroPowerBehavior(behavior);
    }
}