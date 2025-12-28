package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.measure.Angle;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

@TeleOp(name = "All Testing", group = "Experimental")
public class AllTesting extends LinearOpMode
{
    RobotHardware hw;
    InputHandler input;
    MotorExPlus intake;
    ServoEx transfer;
    MotorExPlusGroup outtake;
    Turret turret;
    TransferState transferState = TransferState.MID;

    public void initSystems()
    {
        input = new InputHandler();
        hw = new RobotHardware(hardwareMap, telemetry);
        intake = hw.intake;
        transfer = hw.transfer;
        outtake = hw.getOuttakeMotorExPlusGroup();
        turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE, telemetry);
        bindKeys();

        telemetry.addData("Intake RPM", intake.getRPM());
        telemetry.addData("Servo State", transferState.toString());
        telemetry.addData("Relative Turret Heading", turret.getRelativeHeading(AngleUnit.DEGREES).toNormalUnnormal(AngleUnit.DEGREES));
        telemetry.addData("Outtake RPM", outtake.getRPM());
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
                            transfer.set(0.5);
                        case MID:
                            transferState = TransferState.MAX;
                            transfer.set(1);
                        case MAX:
                            transferState = TransferState.MIN;
                            transfer.set(0);
                    }
                }
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
            run();
        }
    }

    public void run()
    {
        telemetry.addData("Intake RPM", intake.getRPM());
        telemetry.addData("Servo State", transferState.toString());
        telemetry.addData("Relative Turret Heading", turret.getRelativeHeading(AngleUnit.DEGREES).toNormalUnnormal(AngleUnit.DEGREES));
        telemetry.addData("Outtake RPM", outtake.getRPM());
        turret.periodic();
    }
}
