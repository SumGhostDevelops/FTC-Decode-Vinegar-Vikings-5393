package org.firstinspires.ftc.teamcode.definitions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;

public class RobotContext
{
    public final RobotHardware hw; // Raw interface of the robot's hardware
    public final Drive drive; // Drive subsystem
    public final Intake intake; // Intake subsystem
    public final Outtake outtake; // Outtake subsystem
    public final Transfer transfer; // Transfer subsystem

    public final Team team; // Team color the robot is initialized to
    public final Odometry odometry; // Odometry of the robot
    public final Telemetry telemetry; // The robot's telemetry interface
    public final Gamepads gamepads; // A custom class for easily sharing access to the robot's gamepads

    public RobotContext(Team team, HardwareMap hardwareMap, Telemetry telemetry, Gamepads gamepads)
    {
        this.hw = new RobotHardware(hardwareMap, telemetry);
        this.odometry = new Odometry(hw.webcam, hw.pinpoint, RobotConstants.Odometry.DEFAULT_POSE, team.forwardAngle);
        this.drive = new Drive(hw.getDriveArray());
        this.intake = new Intake(hw.intake);
        this.outtake = new Outtake(hw.getOuttakeMotorExPlusGroup());
        this.transfer = new Transfer(hw.transfer);

        this.team = team;
        this.telemetry = telemetry;
        this.gamepads = gamepads;
    }
}