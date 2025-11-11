package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

// This is initialization

public class ControlHub
{
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotorEx launcher;
    public WebcamName camera;
    public IMU imu;
    public CRServo loader;

    MecanumDrive drive;

    public ElapsedTime timer;
    public CRServo BasicServo;
    public Servo RegularServo;

    public void init(HardwareMap map, Pose2d initialPose) {
        leftFront = map.get(DcMotor.class, "leftFront");
        rightFront = map.get(DcMotor.class, "rightFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightBack = map.get(DcMotor.class, "rightBack");
        imu = map.get(IMU.class, "imu");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = map.get(WebcamName.class,"Webcam 1");
        drive = new MecanumDrive(map,initialPose);
        launcher = map.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        loader = map.get(CRServo.class, "Loader");
        loader.setDirection(CRServo.Direction.REVERSE);
    }
}
