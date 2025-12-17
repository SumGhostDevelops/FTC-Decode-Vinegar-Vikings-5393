package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.controls.BlockingCommands;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.modules.Drive;
import org.firstinspires.ftc.teamcode.subsystems.modules.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.modules.Intake;
import org.firstinspires.ftc.teamcode.subsystems.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.modules.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.Odometry;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    protected RobotHardware hw;
    protected Odometry localization;
    protected Drive drive;
    protected Intake intake;
    protected Outtake outtake;
    protected Transfer transfer;
    protected Macros macros;
    protected Gamepads gamepads;

    protected InputHandler input;

    // Assume a PedroPath Follower object would be initialized here
    // protected Follower follower;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Odometry(hw);
        drive = new Drive(hw, localization);
        outtake = new Outtake(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

        gamepads = new Gamepads(gamepad1, gamepad2);

        RobotContext robotContext = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);

        // follower = new Follower(hardwareMap); // PedroPath init

        waitForStart();

        if (opModeIsActive())
        {
            BlockingCommands.sleep(25, "Waiting 25 seconds before moving.");
            drive.setDrivePowers(1, 0, 0);
            macros.sleep(0.4);
            drive.stop();
        }

        localization.close();
    }
}