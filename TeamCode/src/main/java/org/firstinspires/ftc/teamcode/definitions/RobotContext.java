package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.OdometryControlHub;

public class RobotContext
{
    public final RobotHardware hw; // Raw interface of the robot's hardware
    public final Subsystems subsystems;

    public final Team team; // Team color the robot is initialized to
    public final Telemetry telemetry; // The robot's telemetry interface
    public final Gamepads gamepads; // A custom class for easily sharing access to the robot's gamepads

    public RobotContext(Team team, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.hw = new RobotHardware(hardwareMap, telemetry);
        this.subsystems = new Subsystems(hw);

        this.team = team;
        this.telemetry = telemetry;
        this.gamepads = new Gamepads(gamepad1, gamepad2);
    }
}