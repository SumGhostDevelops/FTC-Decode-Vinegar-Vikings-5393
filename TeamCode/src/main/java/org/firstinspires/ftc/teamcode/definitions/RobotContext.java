package org.firstinspires.ftc.teamcode.definitions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;

import java.util.function.Supplier;

public class RobotContext
{
    public final RobotHardware hw;
    public final Drive drive;
    public final Intake intake;
    public final Outtake outtake;
    public final Transfer transfer;

    public final Team team;
    public final Odometry localization;
    public final Telemetry telemetry;
    public final Gamepads gamepads;
    public final Supplier<Boolean> opModeIsActive;

    public RobotContext(Team team, RobotHardware hw, Drive drive, Intake intake, Transfer transfer, Outtake outtake, Odometry localization, Gamepads gamepads, Telemetry telemetry, Supplier<Boolean> opModeIsActive)
    {
        this.hw = hw;
        this.team = team;
        this.drive = drive;
        this.intake = intake;
        this.transfer = transfer;
        this.outtake = outtake;
        this.localization = localization;
        this.gamepads = gamepads;
        this.telemetry = telemetry;
        this.opModeIsActive = opModeIsActive;
    }
}
