package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.measure.Angle;

@TeleOp(name = "Turret Testing (Works)", group = "Experiments")
public class TurretTestingWorks extends LinearOpMode
{
    protected InputHandler input;
    private Turret turret;
    private Odometry odometry;
    private RobotHardware hw;
    private boolean holding = false;
    private Angle holdingAngle = RobotConstants.Turret.FORWARD_ANGLE;

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
        hw = new RobotHardware(hardwareMap, telemetry);
        turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE);
        odometry = new Odometry(hw, telemetry);
        bindKeys();
    }

    protected void bindKeys()
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

        /*
        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> turret.setPower(0.5)
        );
        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () -> turret.setPower(-0.5)
        );
        input.bind(
                () -> (gamepad1.left_trigger > 0.25 && gamepad1.right_trigger > 0.25) || (gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25),
                () -> turret.setPower(0)
        );

         */

        input.bind(
                () -> gamepad1.bWasPressed(),
                () ->
                {
                    hw.turret.setRunMode(Motor.RunMode.RawPower);
                    hw.turret.set(0.25);
                }
        );
        input.bind(
                () -> gamepad1.bWasReleased(),
                () ->
                {
                    hw.turret.setRunMode(Motor.RunMode.RawPower);
                    hw.turret.set(0);
                }
        );

        input.bind(
                () -> gamepad1.yWasPressed(),
                () -> {
                    if (holding)
                    {
                        telemetry.log().add("No longer holding.");
                        holding = false;
                        hw.turret.set(0);
                    }
                    else
                    {
                        holding = true;
                        holdingAngle = odometry.getAngle().toUnit(AngleUnit.DEGREES);
                    }
                }
        );
    }

    protected void run() throws InterruptedException
    {
        telemetry.addData("Robot Heading", odometry.getAngle().toUnit(AngleUnit.DEGREES));
        telemetry.addData("Turret Relative Heading", turret.getRelativeAngle().getUnsignedAngle(AngleUnit.DEGREES));
        telemetry.addData("Raw Ticks", hw.turret.getCurrentPosition());
        telemetry.addData("Target Ticks", turret.getTargetPosition());
        turret.periodic();
        if (holding)
        {
            turret.aimAbsolute(holdingAngle, odometry.getAngle());
        }
    }
}