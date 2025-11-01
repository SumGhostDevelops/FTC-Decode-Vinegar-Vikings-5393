package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class Robot
{
    public ControlHub hub;
    public AprilTagWebcam webcam;
    public Telemetry telemetry;
    public Gamepad gamepad;
    public Supplier<Boolean> opModeIsActive;
    public RobotStatus status;


    public Robot(ControlHub hub, AprilTagWebcam webcam, Telemetry telemetry, Gamepad gamepad, Supplier<Boolean> opModeIsActive, RobotStatus status)
    {
        this.hub = hub;
        this.webcam = webcam;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.opModeIsActive = opModeIsActive;
        this.status = status;
    }
}