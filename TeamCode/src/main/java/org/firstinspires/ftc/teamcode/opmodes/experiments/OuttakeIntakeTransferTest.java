// java
package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

@TeleOp(name = "OuttakeIntakeTransferTest", group = "Experiments")
public class OuttakeIntakeTransferTest extends LinearOpMode {

    private static final double TARGET_RPM = 4200.0;
    private static final double RPM_TOLERANCE = 75.0;
    private static final double OUTTAKE_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_ENGAGED_POS = 0;
    private static final double TRANSFER_DISENGAGED_POS = 180;

    // EMA smoothing factor for RPM (0 < alpha <= 1). Lower = smoother.
    private static final double RPM_ALPHA = 0.25;

    // Stability check: require this many consecutive samples at/above threshold
    private static final int REQUIRED_STABLE_SAMPLES = 3;

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        MotorExPlus outtake = hw.outtakeLeft;
        MotorExPlus intake = hw.intake;
        ServoEx transfer = hw.transfer;

        intake.set(0.0);
        transfer.set(TRANSFER_DISENGAGED_POS);

        boolean startedSequence = false;
        boolean engagedTransfer = false;
        double smoothedRpm = 0.0;

        int stableCount = 0;
        double engageThreshold = TARGET_RPM - RPM_TOLERANCE;

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;

            // read RPM (use MotorExPlus API that provides RPM)
            double currentRpm = outtake.getRPM();

            // initialize/smooth RPM
            if (smoothedRpm == 0.0) {
                smoothedRpm = currentRpm;
            } else {
                smoothedRpm = (RPM_ALPHA * currentRpm) + ((1.0 - RPM_ALPHA) * smoothedRpm);
            }

            // Start spinning outtake when right trigger pressed
            if (rt > 0.5 && !startedSequence) {
                outtake.set(OUTTAKE_POWER);
                startedSequence = true;
                telemetry.addData("Action", "Spinning outtake");
            }

            // When smoothed RPM reaches/ exceeds threshold for a few samples, enable intake and transfer
            if (startedSequence && !engagedTransfer) {
                if (smoothedRpm >= engageThreshold) {
                    stableCount++;
                    if (stableCount >= REQUIRED_STABLE_SAMPLES) {
                        intake.set(INTAKE_POWER);
                        transfer.set(TRANSFER_ENGAGED_POS);
                        engagedTransfer = true;
                        telemetry.addData("Action", "Intake and Transfer engaged");
                    }
                } else {
                    stableCount = 0;
                }
            }

            // Stop everything with left trigger
            if (lt > 0.5) {
                outtake.set(0.0);
                intake.set(0.0);
                transfer.set(TRANSFER_DISENGAGED_POS);
                startedSequence = false;
                engagedTransfer = false;
                smoothedRpm = 0.0;
                stableCount = 0;
                telemetry.addData("Action", "Stopped");
            }

            telemetry.addData("Raw RPM", "%.1f", currentRpm);
            telemetry.addData("Smoothed RPM", "%.1f", smoothedRpm);
            telemetry.addData("Threshold", "%.1f", engageThreshold);
            telemetry.addData("StableCount", stableCount);
            telemetry.addData("At Target", engagedTransfer);
            telemetry.update();

            sleep(50);
        }

        // Ensure stop on exit
        outtake.set(0.0);
        intake.set(0.0);
        transfer.set(TRANSFER_DISENGAGED_POS);
    }
}
