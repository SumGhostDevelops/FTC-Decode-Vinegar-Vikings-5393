package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.controls.Macros;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    protected RobotHardware hw;
    protected Localization localization;
    protected Drive drive;
    protected Intake intake;
    protected Outtake outtake;
    protected Transfer transfer;
    protected Macros macros;
    protected Gamepads gamepads;

    protected InputHandler input;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            macros.update();
            localization.update();
            transfer.update();
            outtake.update();

            telemetry.update();
        }

        localization.close();
    }

    protected void initSystems()
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(hw);
        drive = new Drive(hw, localization);
        outtake = new Outtake(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);
        gamepads = new Gamepads(gamepad1, gamepad2);
        RobotContext robotContext = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);
        macros = new Macros(robotContext);

        input = new InputHandler();
        bindKeys();

        telemetry.setAutoClear(RobotConstants.TELEMETRY_SET_AUTOCLEAR);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();
    }

    protected void run() throws InterruptedException
    {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Drive
        drive.drive(axial, lateral, yaw);

        //telemetry.addData("Drive Mode", drive.getMode());
        telemetry.addData("Team", team);
        telemetry.addLine("\n-----Velocity-----");
        telemetry.addData("Speed", RobotConstants.DRIVE_SPEED_MULTIPLIER);
        telemetry.addData("Heading", localization.getHeading());
        telemetry.addLine("\n-----Outtake-----");
        telemetry.addData("Toggled", outtake.isToggled());
        telemetry.addData("Target RPM", outtake.getTargetRPM());
        telemetry.addData("RPM", outtake.getRPM());
        telemetry.addData("RPM Acceleration", outtake.getRPMAcceleration());
    }

    /*
    public static double getGoalOffset(double range, double bearing, int id, Telemetry telemetry)
    {
        double xDist;
        double yDist;
        double xTargetDist;
        double yTargetDist;
        double degreesToTarget;
        double degreesToTag;
        double degreesToAdd;
        if(id==24)
        {
            xDist = Math.cos(bearing)*range;
            yDist = Math.sin(bearing)*range;
            xTargetDist = xDist + 14;
            yTargetDist = yDist + 12;
            degreesToTarget = Math.tan((yTargetDist /xTargetDist));
            degreesToTag = Math.tan((yDist/xDist));
            degreesToAdd = degreesToTarget-degreesToTag;
        }
        else
        {
            telemetry.log().add("Cancelling auto-aim command: Could not find Tag " + id);
            return 0.0;
        }

        telemetry.log().add("Goal offset: " + -degreesToAdd);
        return -degreesToAdd;
    }
     */

    protected void bindKeys()
    {
        // Handle B
        input.bind
                (
                        () -> gamepad1.bWasPressed(),
                        () -> localization.resetHeading()
                );

        /*
        input.bind
                (
                        () -> gamepad1.xWasPressed(),
                        () -> macros.printRangeToAprilTag()
                );
         */

        input.bind
                (
                        () -> gamepad1.xWasPressed(),
                        () -> outtake.toggleRPM()
                );

        input.bind
                (
                        () -> gamepad1.yWasPressed(),
                        () -> macros.aimToTeamAprilTag()
                );

        /*
        input.bind
                (
                        () -> gamepad1.dpadLeftWasPressed(),
                        () -> RobotConstants.FORCED_ANGLE_OFFSET -= 0.25
                );

        input.bind
                (
                        () -> gamepad1.dpadRightWasPressed(),
                        () -> RobotConstants.FORCED_ANGLE_OFFSET += 0.25
                );
         */

        input.bind
                (
                        () -> gamepad1.leftBumperWasPressed(),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER - RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.max(RobotConstants.DRIVE_SPEED_MINIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );

        input.bind
                (
                        () -> gamepad1.rightBumperWasPressed(),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER + RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.min(RobotConstants.DRIVE_SPEED_MAXIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );

        input.bind
                (
                        () -> gamepad1.left_trigger > 0.25 && outtake.isReadyToLaunch(),
                        () -> transfer.setPower(1)
                );

        input.bind
                (
                        () -> gamepad1.left_trigger <= 0.25 || !outtake.isReadyToLaunch(),
                        () -> transfer.stop()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger > 0.25,
                        () -> outtake.setRPM()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger <= 0.25,
                        () -> outtake.stop()
                );
    }
}