package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.Commands;
import org.firstinspires.ftc.teamcode.controls.Commands.IntakeIn;
import org.firstinspires.ftc.teamcode.controls.Commands.IntakeOut;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;


@TeleOp(name = "Intake Testing", group = "Experiments")
public class IntakeTesting extends LinearOpMode {

    double power = 0.8;

    Intake intake;

    Transfer transfer;

    @Override
    public void runOpMode() {

        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        intake = new Intake(hw.intake);
        transfer = new Transfer(hw.transfer);

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
                new IntakeIn(intake, () -> power)
        );

        intakeOutTrigger.whileActiveOnce(
                new IntakeOut(intake, () -> power)
        );

        transferEngageTrigger.whenActive(
                new Commands.TransferEngage(transfer)
        );

        transferDisengageTrigger.whenActive(
                new Commands.TransferDisengage(transfer)
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
