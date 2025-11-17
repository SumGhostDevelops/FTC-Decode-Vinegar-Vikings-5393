package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.odometry.Webcam;

import java.util.function.Supplier;

public class Robot
{
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx launcher;
    private CRServo loader;

    private IMU imu;
    private Webcam webcam;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Telemetry telemetry;
    private Supplier<Boolean> opModeIsActive;

    public void Robot(HardwareMap map, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Supplier<Boolean> opModeIsActive)
    {
        // Wheels
        this.leftFront = map.get(DcMotor.class, "leftFront");
        this.rightFront = map.get(DcMotor.class, "rightFront");
        this.leftBack = map.get(DcMotor.class, "leftBack");
        this.rightBack = map.get(DcMotor.class, "rightBack");

        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry
        this.imu = map.get(IMU.class, "imu");
        WebcamName cameraName = map.get(WebcamName.class,"Webcam 1");
        double fx = 1424.38;
        double fy = 1424.38;
        double cx = 637.325;
        double cy = 256.77;
        this.webcam = new Webcam(cameraName, fx, fy, cx, cy, true);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.opModeIsActive = opModeIsActive;
    }


}
