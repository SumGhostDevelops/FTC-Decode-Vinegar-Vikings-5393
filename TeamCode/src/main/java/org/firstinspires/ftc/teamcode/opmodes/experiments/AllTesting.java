package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

@TeleOp(name = "All Testing", group = "Experimental")
public class AllTesting extends LinearOpMode
{
    RobotHardware hw;
    InputHandler input;
    MotorExPlus intake;
    ServoEx transfer;
    MotorExPlus outtake;
    Turret turret;
    TransferState transferState = TransferState.MID;

    public void initSystems()
    {
        input = new InputHandler();
        hw = new RobotHardware(hardwareMap, telemetry);
        intake = hw.intake;
        transfer = hw.transfer;
        outtake = hw.outtake;
        turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE);
        turret.lockToPosition = true;
        bindKeys();

        transfer.set(RobotConstants.Transfer.MID_ANGLE);

        telemetry.addData("Intake RPM", intake.getRPM());
        telemetry.addData("Servo State", transferState.toString());
        telemetry.addData("Relative Turret Heading", turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Servo Raw Position", transfer.getRawPosition());
    }

    public enum TransferState
    {
        MIN,
        MID,
        MAX,
    }

    public void bindKeys()
    {
        input.bind(
                () -> gamepad1.dpadUpWasPressed(),
                () -> turret.aimRelative(new Angle(0, AngleUnit.DEGREES))
        );

        input.bind(
                () -> gamepad1.dpadLeftWasPressed(),
                () -> turret.aimRelative(new Angle(90, AngleUnit.DEGREES))
        );

        input.bind(
                () -> gamepad1.dpadDownWasPressed(),
                () -> turret.aimRelative(new Angle(180, AngleUnit.DEGREES))
        );

        input.bind(
                () -> gamepad1.dpadRightWasPressed(),
                () -> turret.aimRelative(new Angle(270, AngleUnit.DEGREES))
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> hw.intake.set(0.5)
        );

        input.bind(
                () -> gamepad1.left_trigger < 0.25,
                () -> hw.intake.set(0)
        );

        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () ->
                {
                    outtake.setRunMode(Motor.RunMode.RawPower);
                    outtake.set(0.8);
                }
        );

        input.bind(
                () -> gamepad1.right_trigger < 0.25,
                () ->
                {
                    outtake.setRunMode(Motor.RunMode.RawPower);
                    outtake.set(0);
                }
        );

        input.bind(
                () -> gamepad1.aWasPressed(),
                () ->
                {
                    switch (transferState)
                    {
                        case MIN:
                            transferState = TransferState.MID;
                            transfer.set(RobotConstants.Transfer.MID_ANGLE);
                            break;
                        case MID:
                            transferState = TransferState.MAX;
                            transfer.set(RobotConstants.Transfer.CLOSED_ANGLE);
                            break;
                        case MAX:
                            transferState = TransferState.MIN;
                            transfer.set(RobotConstants.Transfer.OPEN_ANGLE);
                            break;
                    }
                }
        );

        input.bind(
                () -> gamepad1.bWasPressed(),
                () -> transfer.set(0)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initSystems();

        waitForStart();

        while (opModeIsActive())
        {
            input.update();
            telemetry.update();
            run();
        }
    }

    public void run()
    {
        telemetry.addData("Intake RPM", intake.getRPM());
        telemetry.addData("Servo State", transferState.toString());
        telemetry.addData("Relative Turret Heading", turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Servo Raw Position", transfer.getRawPosition());
        turret.periodic();
    }
}
