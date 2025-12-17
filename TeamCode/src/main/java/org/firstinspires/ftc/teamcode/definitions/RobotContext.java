package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.modules.Drive;
import org.firstinspires.ftc.teamcode.subsystems.modules.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.modules.Intake;
import org.firstinspires.ftc.teamcode.subsystems.modules.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.modules.outtake.Outtake;

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

    public RobotContext(Team team, HardwareMap hardwareMap, Telemetry telemetry, Gamepads gamepads)
    {
        this.hw = new RobotHardware(hardwareMap);
        this.localization = new Odometry(hw, telemetry);
        this.drive = new Drive(hw, localization, telemetry);
        this.intake = new Intake(hw);
        this.outtake = new Outtake(hw);
        this.transfer = new Transfer(hw);

        this.team = team;
        this.telemetry = telemetry;
        this.gamepads = gamepads;
    }
}