package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

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
        this.hw = new RobotHardware(hardwareMap, telemetry);
        this.localization = new Odometry(hw, telemetry);
        this.drive = new Drive(hw.getDriveArray());
        this.intake = new Intake(hw.intake);
        this.outtake = new Outtake(hw.getOuttakeMotorExPlusGroup());
        this.transfer = new Transfer(hw.transfer);

        this.team = team;
        this.telemetry = telemetry;
        this.gamepads = gamepads;
    }
}