package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

@TeleOp(name = "Flywheel PIDF Tuning", group = "Experiments")
public class FlywheelPIDFTuning extends Base
{
    private enum Coefficient {P, I, D, F}

    private Coefficient coefficient = Coefficient.P;
    private double p, i, d, f;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }

    @Override
    protected void run() throws InterruptedException
    {
        super.run();

        telemetry.addData("Modifying Coefficient", coefficient);
        PIDFCoefficients cfs = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry.addData("PIDF (Expected/Actual)", "(" + p + "/" + cfs.p + ") (" + i + "/" + cfs.i + ") (" + d + "/" + cfs.d + ") (" + f + "/" + cfs.f + ") (");

        telemetry.update();
    }

    private void varyPIDF(double change)
    {
        switch (coefficient)
        {
            case P:
                p += change;
                break;
            case I:
                i += change;
                break;
            case D:
                d += change;
                break;
            case F:
                f += change;
                break;
        }

        PIDFCoefficients newCoeffs = new PIDFCoefficients(p, i, d, f);
        hw.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);
    }

    @Override
    protected void bindKeys()
    {
        super.bindKeys();

        input.bind
                (
                        () -> gamepad2.yWasPressed(),
                        () -> coefficient = Coefficient.P
                );

        input.bind
                (
                        () -> gamepad2.xWasPressed(),
                        () -> coefficient = Coefficient.I
                );

        input.bind
                (
                        () -> gamepad2.bWasPressed(),
                        () -> coefficient = Coefficient.D
                );

        input.bind
                (
                        () -> gamepad2.aWasPressed(),
                        () -> coefficient = Coefficient.F
                );

        input.bind
                (
                        () -> gamepad2.dpadUpWasPressed(),
                        () -> varyPIDF(0.05)
                );

        input.bind
                (
                        () -> gamepad2.dpadDownWasPressed(),
                        () -> varyPIDF(-0.05)
                );
    }
}
