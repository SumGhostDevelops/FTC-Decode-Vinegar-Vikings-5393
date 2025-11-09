package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class RobotContext
{
    public ControlHub hub;
    public AprilTagWebcam webcam;
    public Telemetry telemetry;
    public Gamepad gamepad;
    public Supplier<Boolean> opModeIsActive;
    public Robot self;
    public Wheels wheels;
    public Launcher launcher;


    public RobotContext(ControlHub hub, AprilTagWebcam webcam, Telemetry telemetry, Gamepad gamepad, Supplier<Boolean> opModeIsActive, Robot self, Wheels wheels, Launcher launcher)
    {
        this.hub = hub;
        this.webcam = webcam;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.opModeIsActive = opModeIsActive;
        this.self = self;
        this.wheels = wheels;
        this.launcher = launcher;
    }
}