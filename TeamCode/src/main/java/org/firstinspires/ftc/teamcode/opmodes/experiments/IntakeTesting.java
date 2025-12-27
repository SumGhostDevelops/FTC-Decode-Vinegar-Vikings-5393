package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

@TeleOp(name = "Intake Testing", group = "Experiments")
public class IntakeTesting extends LinearOpMode
{
    protected InputHandler input;
    double power = 0.5;
    MotorExPlus intake;
    @Override
    public void runOpMode() throws InterruptedException
    {

        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            telemetry.update();
        }
    }

    protected void initSystems()
    {
        input = new InputHandler();
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        intake = hw.intake;
        bindKeys();
    }

    protected void bindKeys()
    {
        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () -> intake.set(power)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> intake.set(-power)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25 && gamepad1.right_trigger > 0.25 || gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25,
                () -> intake.set(0)
        );

        input.bind(
                () -> gamepad1.dpadUpWasPressed(),
                () -> power += 0.1
        );

        input.bind(
                () -> gamepad1.dpadDownWasPressed(),
                () -> power -= 0.1
        );
    }

    protected void run() throws InterruptedException
    {
        telemetry.addData("Power", power);
        telemetry.addData("Motor RPM", intake.getRPM());
    }
}
