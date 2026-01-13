package org.firstinspires.ftc.teamcode.definitions.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.controls.Gamepads;

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
        this.subsystems = new Subsystems(hw, team);

        this.team = team;
        this.telemetry = telemetry;
        this.gamepads = new Gamepads(gamepad1, gamepad2);
    }
}