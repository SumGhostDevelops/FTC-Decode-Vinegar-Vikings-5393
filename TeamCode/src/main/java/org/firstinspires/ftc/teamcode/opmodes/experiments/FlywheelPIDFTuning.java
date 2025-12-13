package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

@TeleOp(name = "Flywheel PIDF Tuning", group = "Experiments")
public class FlywheelPIDFTuning extends Base
{
    private enum Coefficient {P, I, D, F}

    private Coefficient coefficient = Coefficient.P;
    private double p, i, d, f;
    private double coefficientChange = 1;
    private boolean firstRun = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }

    @Override
    protected void run() throws InterruptedException
    {
        input.update();
        if (firstRun)
        {
            //hw.outtakeMotor.setVelocityPIDFCoefficients(1, 1, 1, 1);
            PIDFCoefficients cfs = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            p = cfs.p;
            i = cfs.i;
            d = cfs.d;
            f = cfs.f;
            firstRun = false;
        }

        telemetry.addData("Outtake Target RPM", outtake.getTargetRPM());
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Outtake RPM Acceleration", outtake.getRPMAcceleration());
        telemetry.addData("Coefficient Change", coefficientChange);
        telemetry.addData("Modifying Coefficient", coefficient);
        PIDFCoefficients cfs = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry.addData("PIDF (Expected/Actual)", "(" + p + "/" + cfs.p + ") (" + i + "/" + cfs.i + ") (" + d + "/" + cfs.d + ") (" + f + "/" + cfs.f + ")");

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
        telemetry.log().add("Changed " + coefficient + " by " + change);
    }

    @Override
    protected void bindKeys()
    {
        telemetry.log().add("Bind keys ran.");

        input.bind
                (
                        () -> gamepad1.yWasPressed(),
                        () -> coefficient = Coefficient.P
                );

        input.bind
                (
                        () -> gamepad1.xWasPressed(),
                        () -> coefficient = Coefficient.I
                );

        input.bind
                (
                        () -> gamepad1.bWasPressed(),
                        () -> coefficient = Coefficient.D
                );

        input.bind
                (
                        () -> gamepad1.aWasPressed(),
                        () -> coefficient = Coefficient.F
                );

        input.bind
                (
                        () -> gamepad1.dpadUpWasPressed(),
                        () -> varyPIDF(coefficientChange)
                );

        input.bind
                (
                        () -> gamepad1.dpadDownWasPressed(),
                        () -> varyPIDF(-coefficientChange)
                );

        input.bind
                (
                        () -> gamepad1.dpadLeftWasPressed(),
                        () -> coefficientChange -= 0.05
                );

        input.bind
                (
                        () -> gamepad1.dpadRightWasPressed(),
                        () -> coefficientChange += 0.05
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
