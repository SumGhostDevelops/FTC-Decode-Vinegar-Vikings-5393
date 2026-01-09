package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.controls.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;


@TeleOp(name = "Intake Testing", group = "Experiments")
@Disabled
public class IntakeTesting extends LinearOpMode {

    double power = 0.8;

    org.firstinspires.ftc.teamcode.subsystems.Intake intake;

    org.firstinspires.ftc.teamcode.subsystems.Transfer transfer;

    @Override
    public void runOpMode() {

        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        intake = new org.firstinspires.ftc.teamcode.subsystems.Intake(hw.intake);
        transfer = new org.firstinspires.ftc.teamcode.subsystems.Transfer(hw.transfer);

        // Triggers
        Trigger intakeInTrigger =
                new Trigger(() -> gamepad1.right_trigger > 0.25);

        Trigger intakeOutTrigger =
                new Trigger(() -> gamepad1.left_trigger > 0.25);

        Trigger transferEngageTrigger =
                new Trigger(() -> gamepad1.dpad_up);

        Trigger transferDisengageTrigger =
                new Trigger(() -> gamepad1.dpad_down);


        intakeInTrigger.whileActiveOnce(
                new IntakeCommands.In(intake, () -> power)
        );

        intakeOutTrigger.whileActiveOnce(
                new IntakeCommands.Out(intake, () -> power)
        );

        transferEngageTrigger.whenActive(
                new TransferCommands.OpenTransfer(transfer)
        );

        transferDisengageTrigger.cancelWhenActive(
                new TransferCommands.OpenTransfer(transfer)
        );




        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("Power", power);
            telemetry.addData("RPM", intake.getRPM());
            telemetry.update();
        }
    }
}
