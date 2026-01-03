// java
package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name = "OuttakeIntakeTransferTest", group = "Experiments")
public class OuttakeIntakeTransferTest extends LinearOpMode {

    protected InputHandler input;
    double power = 0.8;
    double rpm = 3500;
    double accelerationTolerance = 8;
    double RPMTolerance = 50;
    boolean hasLaunched = false;
    Intake intake;
    Transfer transfer;
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        initSystems();
        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();
            launch();
            resetLaunch();
            telemetry.update();
        }
    }

    protected void initSystems()
    {
        input = new InputHandler();
        bindKeys();
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        intake = new Intake(hw.intake);
        transfer = new Transfer(hw.transfer);
        outtake = new Outtake(hw.getOuttakeMotorExPlusGroup());
    }
    protected void launch() throws InterruptedException {
        if(atSpeed() && !hasLaunched) {
            intake.in(power);
            Thread.sleep(100);
            transfer.flip();
            hasLaunched = true;
        }
    }
    protected void resetLaunch()
    {
        if(!atSpeed()){
            hasLaunched=false;
        }
    }
    protected boolean atSpeed()
    {
        return((Math.abs(outtake.getAcceleration()) <= accelerationTolerance)&&(Math.abs(outtake.getVelocity() - rpm) <= RPMTolerance));
    }

    protected void run() throws InterruptedException
    {
        telemetry.addData("Outtake Target RPM", rpm);
        telemetry.addData("Outtake RPM", outtake.getVelocity());

        telemetry.addData("Intake Power", power);
        telemetry.addData("Intake RPM", intake.getRPM());

    }

    protected void bindKeys()
    {
        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () -> outtake.setRPM(rpm)
        );
        input.bind(
                () -> gamepad1.right_trigger < 0.25,
                () -> outtake.stop()
        );
        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> intake.in(power)
        );
        input.bind(
                () -> gamepad1.left_trigger < 0.25,
                () -> intake.in(0)
        );
        input.bind(
                () -> gamepad1.yWasPressed(),
                () -> power = -power
        );
        input.bind(
                () -> gamepad1.xWasPressed(),
                () -> transfer.close()
        );
        input.bind(
                () -> gamepad1.aWasPressed(),
                () -> transfer.flip()
        );
        input.bind(
                () -> gamepad1.bWasPressed(),
                () -> transfer.open()
        );
        input.bind(
                () -> gamepad1.dpadUpWasPressed(),
                () -> rpm+=25
        );
        input.bind(
                () -> gamepad1.dpadDownWasPressed(),
                () -> rpm+=25
        );
        input.bind(
                () -> gamepad1.dpadRightWasPressed(),
                () -> power+=0.05
        );
        input.bind(
                () -> gamepad1.dpadLeftWasPressed(),
                () -> power-=0.05
        );
    }

}
