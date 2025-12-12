package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    private RobotHardware hw;
    private Localization localization;
    private Drive drive;
    private Intake intake;
    private Outtake outtake;
    private Transfer transfer;
    private Macros macros;
    private Gamepads gamepads;

    private InputHandler input;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(hw);
        drive = new Drive(hw, localization);
        outtake = new Outtake(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();

        gamepads = new Gamepads(gamepad1, gamepad2);

        RobotContext robotContext = new RobotContext(team, hw, drive, intake, transfer, outtake, localization, gamepads, telemetry, this::opModeIsActive);

        macros = new Macros(robotContext);

        input = new InputHandler();
        bindKeys();

        waitForStart();

        while (opModeIsActive())
        {
            run();
        }

        localization.close();
    }

    private void run() throws InterruptedException
    {
        input.update();

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Drive
        drive.drive(axial, lateral, yaw);

        telemetry.addData("Drive Mode", drive.getMode());
        telemetry.addData("Speed", RobotConstants.DRIVE_SPEED_MULTIPLIER);
        telemetry.addData("Heading", localization.getHeading());
        telemetry.addData("Outtake Target RPM", outtake.getTargetRPM());
        telemetry.addData("Outtake RPM", outtake.getRPM());
        PIDFCoefficients coefficients = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF", coefficients.p + " " + coefficients.i + " " + coefficients.d + " " + coefficients.f);
        telemetry.addData("Outtake RPM Acceleration", outtake.getRPMAcceleration());

        macros.update();
        localization.update();
        transfer.update();
        outtake.update();

        telemetry.update();
    }

    private void bindKeys()
    {
        // Handle A
        input.bind
                (
                        () -> gamepad1.aWasPressed(),
                        () -> macros.resetTransferOuttakeNonFSM()
                );
        input.bind
                (
                        () -> gamepad1.aWasReleased(),
                        () -> macros.stopResetTransferOuttakeNonFSM()
                );

        // Handle B
        input.bind
                (
                        () -> gamepad1.bWasPressed(),
                        () -> localization.resetHeading()
                );

        input.bind
                (
                        () -> gamepad1.xWasPressed(),
                        () -> macros.printRangeToAprilTag()
                );

        input.bind
                (
                        () -> gamepad1.yWasPressed(),
                        () -> macros.aimToTeamAprilTag()
                );

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
                        () -> gamepad1.left_trigger < 0.25 || !outtake.isReadyToLaunch(),
                        () -> transfer.stop()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger > 0.25,
                        () -> outtake.setRPM()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger < 0.25,
                        () -> outtake.stop()
                );
    }
}