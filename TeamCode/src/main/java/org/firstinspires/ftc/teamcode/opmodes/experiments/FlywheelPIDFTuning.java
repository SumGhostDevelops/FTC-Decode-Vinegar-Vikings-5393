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
        input.update();

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Drive
        drive.drive(axial, lateral, yaw);

        telemetry.addData("Team", team.color);
        telemetry.addData("Drive Mode", drive.getMode());
        telemetry.addData("Speed", RobotConstants.DRIVE_SPEED_MULTIPLIER);
        telemetry.addData("Heading", localization.getHeading());
        telemetry.addData("Outtake Target RPM", outtake.getTargetRPM());
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Outtake RPM Acceleration", outtake.getRPMAcceleration());
        telemetry.addData("Modifying Coefficient", coefficient);
        PIDFCoefficients cfs = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry.addData("PIDF (Expected/Actual)", "(" + p + "/" + cfs.p + ") (" + i + "/" + cfs.i + ") (" + d + "/" + cfs.d + ") (" + f + "/" + cfs.f + ") (");

        macros.update();
        localization.update();
        transfer.update();
        outtake.update();

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

        PIDFCoefficients current = hw.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        p = current.p;
        i = current.i;
        d = current.d;
        f = current.f;

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
