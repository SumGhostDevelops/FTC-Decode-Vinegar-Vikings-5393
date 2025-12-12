package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardware
{
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx outtakeMotor, intakeMotor;
    public CRServo transferServo;
    public IMU imu;
    public WebcamName webcam;

    // Dead wheel encoders (often accessed via specific motor ports)
    public DcMotorEx parEncoder, perpEncoder;

    public Telemetry telemetry;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        // --- Drive Motors ---
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Directions (Adjust to ensure all wheels spin forward when commanded positive)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, rightFront, leftBack, rightBack);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, leftFront, rightFront, leftBack, rightBack);

        // --- Mechanisms ---
        try
        {
            outtakeMotor = hardwareMap.get(DcMotorEx.class, "launcher");
            outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // For Velocity Control
            outtakeMotor.setVelocityPIDFCoefficients(10, 4, 5, -3);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Launcher not found");
        }

        try
        {
            transferServo = hardwareMap.get(CRServo.class, "transfer");
            transferServo.setDirection(CRServo.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Loader not found");
        }

        try
        {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Intake placeholder not found");
        }

        // --- Sensors ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        webcam = hardwareMap.get(WebcamName.class, "webcam");

        // --- Odometry Encoders ---
        // Assuming they are plugged into specific slots, e.g., parallel on 'par', perpendicular on 'perp'
        // Note: In many configs these are just specific drive motor ports.
        try
        {
            parEncoder = hardwareMap.get(DcMotorEx.class, "par");
            perpEncoder = hardwareMap.get(DcMotorEx.class, "perp");
        }
        catch (Exception e)
        {
            telemetry.log().add("Warning: Dead wheel encoders not mapped");
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