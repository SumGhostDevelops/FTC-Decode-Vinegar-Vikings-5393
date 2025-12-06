package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Webcam;

import java.util.function.Supplier;

public class RobotContext
{
    public Team team;
    public RobotHardware hw;
    public Webcam webcam;
    public Telemetry telemetry;
    public Gamepads gamepads;
    public Supplier<Boolean> opModeIsActive;

    public Drive drive;
    public Intake intake;
    public Outtake outtake;
    public Transfer transfer;
    public Localization localization;

    public RobotContext(Team team, RobotHardware hw, Drive drive, Intake intake, Transfer transfer, Outtake outtake, Webcam webcam, Localization localization, Gamepads gamepads, Telemetry telemetry, Supplier<Boolean> opModeIsActive)
    {
        this.hw = hw;
        this.drive = drive;
        this.intake = intake;
        this.transfer = transfer;
        this.outtake = outtake;
        this.webcam = webcam;
        this.gamepads = gamepads;
        this.telemetry = telemetry;
        this.opModeIsActive = opModeIsActive;
        this.localization = localization;
    }
}
