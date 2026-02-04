package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.util.motors.PositionMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.motors.VelocityMotorGroup;

import java.util.ArrayList;
import java.util.List;

/**
 * PIDF Tuning TeleOp Mode.
 * 
 * Allows interactive tuning of PIDF coefficients for the turret (position
 * control)
 * and outtake (velocity control) subsystems.
 * 
 * Controls:
 * - A/B: Switch between Turret and Outtake tuning modes
 * - DPAD Left/Right: Select which coefficient (P, I, D, F) to adjust
 * - DPAD Up/Down: Increase/Decrease the selected coefficient
 * - Left/Right Bumper: Adjust step size (coarse/fine)
 * - X: Run test movement (turret: 90°, outtake: target RPM)
 * - Y: Reset target to zero / stop outtake
 * - Left Stick Y (Turret mode): Manual turret position control
 * - Right Trigger (Outtake mode): Set target RPM (0-6000)
 * - Start: Print suggested PIDF values based on current tuning
 * - Back: Reset PIDF to default values from RobotConstants
 */
@TeleOp(name = "PIDF Tuner", group = "Utility")
public class PIDFTuner extends OpMode
{
    // Subsystem selection
    private enum TuningMode
    {
        TURRET, OUTTAKE
    }

    private TuningMode currentMode = TuningMode.TURRET;

    // Coefficient selection for adjustment
    private enum Coefficient
    {
        P, I, D, F
    }

    private Coefficient selectedCoeff = Coefficient.P;

    // Hardware
    private PositionMotor turret;
    private VelocityMotorGroup outtake;
    private VoltageSensor battery;
    private double cachedVoltage = 12.0;

    // Current PIDF values
    private PIDFCoefficients turretPIDF;
    private PIDFCoefficients outtakePIDF;

    // Step sizes for adjustment
    private final double[] stepSizes =
    { 0.0001, 0.001, 0.01, 0.1, 1.0 };
    private int stepIndex = 2; // Start at 0.01

    // Telemetry data collection
    private final List<Double> errorHistory = new ArrayList<>();
    private final List<Double> responseHistory = new ArrayList<>();
    private double peakOvershoot = 0;
    private double settlingTime = 0;
    private double steadyStateError = 0;
    private boolean isTestRunning = false;
    private ElapsedTime testTimer = new ElapsedTime();
    private double testStartValue = 0;
    private double testTargetValue = 0;

    // Button debouncing
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB, prevRB, prevStart, prevBack;

    // Constants from RobotConstants
    private final String turretName = RobotConstants.Turret.NAME;
    private final double turretGearRatio = RobotConstants.Turret.GEAR_RATIO;
    private final String outtakeName = RobotConstants.Outtake.Name.LAUNCHER_LEFT;

    @Override
    public void init()
    {
        telemetry.addLine("Initializing PIDF Tuner...");
        telemetry.update();

        try
        {
            battery = hardwareMap.voltageSensor.iterator().next();
            cachedVoltage = battery.getVoltage();
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Voltage sensor not found");
        }

        // Initialize Turret
        try
        {
            turret = new PositionMotor(new MotorEx(hardwareMap, turretName, Motor.GoBILDA.RPM_435), () -> cachedVoltage)
                    .setVoltageCompensation(12)
                    .usePower(1)
                    .setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
                    .setDistancePerPulse(1.0, turretGearRatio, AngleUnit.DEGREES)
                    .setControllerType(PositionMotor.PositionController.SquIDF);

            turretPIDF = new PIDFCoefficients(
                    RobotConstants.Turret.PIDF.p,
                    RobotConstants.Turret.PIDF.i,
                    RobotConstants.Turret.PIDF.d,
                    RobotConstants.Turret.PIDF.f);
            turret.setPIDF(turretPIDF);
            turret.setPositionTolerance(RobotConstants.Turret.TOLERANCE.getDegrees());
            turret.setTargetDistance(0);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Turret motor not found - " + e.getMessage());
        }

        // Initialize Outtake
        try
        {
            VelocityMotor outtakeMotor = new VelocityMotor(
                    new MotorEx(hardwareMap, outtakeName, Motor.GoBILDA.BARE), () -> cachedVoltage);
            outtake = new VelocityMotorGroup(outtakeMotor)
                    .setVoltageCompensation(12)
                    .setControllerType(VelocityMotor.VelocityController.TakeBackHalf);

            outtakePIDF = new PIDFCoefficients(
                    RobotConstants.Outtake.Coefficients.PIDF.p,
                    RobotConstants.Outtake.Coefficients.PIDF.i,
                    RobotConstants.Outtake.Coefficients.PIDF.d,
                    RobotConstants.Outtake.Coefficients.PIDF.f);
            outtake.setPIDF(outtakePIDF);
            outtake.setTolerance(
                    RobotConstants.Outtake.Tolerance.RPM,
                    RobotConstants.Outtake.Tolerance.RPM_ACCELERATION);
        } catch (Exception e)
        {
            telemetry.log().add("Warning: Outtake motor not found - " + e.getMessage());
        }

        telemetry.addLine("PIDF Tuner Ready!");
        telemetry.addLine("Press A for Turret, B for Outtake");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        if (battery != null)
            cachedVoltage = battery.getVoltage();
        handleInput();
        updateHardware();
        collectTuningData();
        displayTelemetry();
    }

    private void handleInput()
    {
        // Mode selection
        if (gamepad1.a && !prevA)
        {
            currentMode = TuningMode.TURRET;
            resetTestData();
        }
        if (gamepad1.b && !prevB)
        {
            currentMode = TuningMode.OUTTAKE;
            resetTestData();
        }

        // Coefficient selection
        if (gamepad1.dpad_left && !prevDpadLeft)
        {
            selectedCoeff = Coefficient.values()[(selectedCoeff.ordinal() - 1 + Coefficient.values().length) % Coefficient.values().length];
        }
        if (gamepad1.dpad_right && !prevDpadRight)
        {
            selectedCoeff = Coefficient.values()[(selectedCoeff.ordinal() + 1) % Coefficient.values().length];
        }

        // Step size adjustment
        if (gamepad1.left_bumper && !prevLB)
        {
            stepIndex = Math.max(0, stepIndex - 1);
        }
        if (gamepad1.right_bumper && !prevRB)
        {
            stepIndex = Math.min(stepSizes.length - 1, stepIndex + 1);
        }

        // Coefficient adjustment
        if (gamepad1.dpad_up && !prevDpadUp)
        {
            adjustCoefficient(stepSizes[stepIndex]);
        }
        if (gamepad1.dpad_down && !prevDpadDown)
        {
            adjustCoefficient(-stepSizes[stepIndex]);
        }

        // Test movement
        if (gamepad1.x && !prevX)
        {
            startTest();
        }
        if (gamepad1.y && !prevY)
        {
            stopTest();
        }

        // Tuning suggestions
        if (gamepad1.start && !prevStart)
        {
            printTuningSuggestions();
        }

        // Reset to defaults
        if (gamepad1.back && !prevBack)
        {
            resetToDefaults();
        }

        // Manual control
        if (currentMode == TuningMode.TURRET && turret != null)
        {
            double stickY = -gamepad1.left_stick_y;
            if (Math.abs(stickY) > 0.1)
            {
                double targetPos = turret.getDistance() + (stickY * 10); // 10 degrees per full stick
                turret.setTargetDistance(targetPos);
            }
        }

        if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            double rpmTarget = gamepad1.right_trigger * 6000;
            if (rpmTarget > 100)
            {
                outtake.setTargetRPM(rpmTarget);
            }
        }

        // Update prev states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLB = gamepad1.left_bumper;
        prevRB = gamepad1.right_bumper;
        prevStart = gamepad1.start;
        prevBack = gamepad1.back;
    }

    private void adjustCoefficient(double delta)
    {
        PIDFCoefficients pidf = (currentMode == TuningMode.TURRET) ? turretPIDF : outtakePIDF;

        switch (selectedCoeff)
        {
            case P:
                pidf.p = Math.max(0, pidf.p + delta);
                break;
            case I:
                pidf.i = Math.max(0, pidf.i + delta);
                break;
            case D:
                pidf.d = Math.max(0, pidf.d + delta);
                break;
            case F:
                pidf.f = Math.max(0, pidf.f + delta);
                break;
        }

        applyPIDF();
    }

    private void applyPIDF()
    {
        if (currentMode == TuningMode.TURRET && turret != null)
        {
            turret.setPIDF(turretPIDF);
        }
        else if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            outtake.setPIDF(outtakePIDF);
        }
    }

    private void startTest()
    {
        resetTestData();
        isTestRunning = true;
        testTimer.reset();

        if (currentMode == TuningMode.TURRET && turret != null)
        {
            testStartValue = turret.getDistance();
            testTargetValue = testStartValue + 90; // 90 degree test movement
            turret.setTargetDistance(testTargetValue);
        }
        else if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            testStartValue = outtake.getRPM();
            testTargetValue = RobotConstants.Outtake.BASE_RPM; // Use base RPM as target
            outtake.setTargetRPM(testTargetValue);
        }
    }

    private void stopTest()
    {
        isTestRunning = false;

        if (currentMode == TuningMode.TURRET && turret != null)
        {
            turret.setTargetDistance(turret.getDistance());
        }
        else if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            outtake.stopMotor();
        }
    }

    private void resetTestData()
    {
        errorHistory.clear();
        responseHistory.clear();
        peakOvershoot = 0;
        settlingTime = 0;
        steadyStateError = 0;
        isTestRunning = false;
    }

    private void resetToDefaults()
    {
        if (currentMode == TuningMode.TURRET)
        {
            turretPIDF = new PIDFCoefficients(
                    RobotConstants.Turret.PIDF.p,
                    RobotConstants.Turret.PIDF.i,
                    RobotConstants.Turret.PIDF.d,
                    RobotConstants.Turret.PIDF.f);
        }
        else
        {
            outtakePIDF = new PIDFCoefficients(
                    RobotConstants.Outtake.Coefficients.PIDF.p,
                    RobotConstants.Outtake.Coefficients.PIDF.i,
                    RobotConstants.Outtake.Coefficients.PIDF.d,
                    RobotConstants.Outtake.Coefficients.PIDF.f);
        }
        applyPIDF();
        telemetry.log().add("PIDF reset to defaults");
    }

    private void updateHardware()
    {
        if (turret != null)
        {
            turret.update();
        }
        if (outtake != null)
        {
            outtake.update();
        }
    }

    private void collectTuningData()
    {
        if (!isTestRunning)
            return;

        double currentValue, error;

        if (currentMode == TuningMode.TURRET && turret != null)
        {
            currentValue = turret.getDistance();
            error = testTargetValue - currentValue;
        }
        else if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            currentValue = outtake.getRPM();
            error = testTargetValue - currentValue;
        }
        else
        {
            return;
        }

        errorHistory.add(error);
        responseHistory.add(currentValue);

        // Calculate overshoot (if we go past target)
        double overshoot = currentValue - testTargetValue;
        if (Math.signum(testTargetValue - testStartValue) == Math.signum(overshoot))
        {
            peakOvershoot = Math.max(peakOvershoot, Math.abs(overshoot));
        }

        // Calculate settling time (time to stay within tolerance)
        double tolerance = (currentMode == TuningMode.TURRET)
                ? RobotConstants.Turret.TOLERANCE.getDegrees()
                : RobotConstants.Outtake.Tolerance.RPM;

        if (Math.abs(error) < tolerance && settlingTime == 0)
        {
            settlingTime = testTimer.seconds();
        }

        // Track steady state error (average of last 10 samples)
        if (responseHistory.size() > 20)
        {
            double sum = 0;
            for (int i = errorHistory.size() - 10; i < errorHistory.size(); i++)
            {
                sum += Math.abs(errorHistory.get(i));
            }
            steadyStateError = sum / 10;
        }
    }

    private void printTuningSuggestions()
    {
        telemetry.log().clear();
        telemetry.log().add("=== TUNING SUGGESTIONS ===");

        PIDFCoefficients pidf = (currentMode == TuningMode.TURRET) ? turretPIDF : outtakePIDF;
        String subsystem = (currentMode == TuningMode.TURRET) ? "Turret" : "Outtake";
        String controller = (currentMode == TuningMode.TURRET) ? "SquIDF" : "TakeBackHalf";

        telemetry.log().add(subsystem + " (" + controller + ")");
        telemetry.log().add(String.format("Current: P=%.5f I=%.5f D=%.5f F=%.5f",
                pidf.p, pidf.i, pidf.d, pidf.f));

        if (errorHistory.size() < 10)
        {
            telemetry.log().add("Run a test (X) to get suggestions!");
            return;
        }

        // Analyze response and provide suggestions
        StringBuilder suggestions = new StringBuilder();

        if (currentMode == TuningMode.TURRET)
        {
            // Position control analysis
            double tolerance = RobotConstants.Turret.TOLERANCE.getDegrees();

            if (steadyStateError > tolerance * 2)
            {
                suggestions.append("• Steady-state error high → ↑ P or ↑ I\n");
            }
            else if (steadyStateError < tolerance * 0.5 && peakOvershoot > tolerance)
            {
                suggestions.append("• Overshoot detected → ↓ P or ↑ D\n");
            }

            if (peakOvershoot > 10)
            { // More than 10 degrees overshoot
                suggestions.append("• Significant overshoot (");
                suggestions.append(String.format("%.1f°", peakOvershoot));
                suggestions.append(") → ↑ D or ↓ P\n");
            }

            if (settlingTime > 1.0)
            {
                suggestions.append("• Slow settling (");
                suggestions.append(String.format("%.2fs", settlingTime));
                suggestions.append(") → ↑ P slightly\n");
            }
            else if (settlingTime < 0.3 && peakOvershoot < 5)
            {
                suggestions.append("• Good response! Consider raising P slightly for faster response\n");
            }

            // Suggest new values
            double suggestedP = pidf.p;
            double suggestedD = pidf.d;

            if (peakOvershoot > 10)
            {
                suggestedP *= 0.8;
                suggestedD *= 1.5;
            }
            else if (steadyStateError > tolerance)
            {
                suggestedP *= 1.2;
            }

            telemetry.log().add(String.format("Suggested P=%.5f D=%.5f", suggestedP, suggestedD));
        }
        else
        {
            // Velocity control analysis (TakeBackHalf uses I primarily)
            double tolerance = RobotConstants.Outtake.Tolerance.RPM;

            if (steadyStateError > tolerance)
            {
                suggestions.append("• RPM not reaching target → ↑ I\n");
            }

            if (peakOvershoot > 200)
            { // More than 200 RPM overshoot
                suggestions.append("• RPM overshoot (");
                suggestions.append(String.format("%.0f", peakOvershoot));
                suggestions.append(") → ↓ I\n");
            }

            if (settlingTime > 2.0)
            {
                suggestions.append("• Slow spinup (");
                suggestions.append(String.format("%.2fs", settlingTime));
                suggestions.append(") → ↑ I\n");
            }

            // For TakeBackHalf, I is the main coefficient
            double suggestedI = pidf.i;
            if (peakOvershoot > 200)
            {
                suggestedI *= 0.8;
            }
            else if (steadyStateError > tolerance)
            {
                suggestedI *= 1.2;
            }

            telemetry.log().add(String.format("Suggested I=%.5f", suggestedI));
        }

        if (suggestions.length() > 0)
        {
            for (String line : suggestions.toString().split("\n"))
            {
                if (!line.isEmpty())
                {
                    telemetry.log().add(line);
                }
            }
        }
        else
        {
            telemetry.log().add("Response looks good! Fine-tune as needed.");
        }
    }

    private void displayTelemetry()
    {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Selected", selectedCoeff + " (DPAD L/R)");
        telemetry.addData("Step Size", stepSizes[stepIndex] + " (LB/RB)");

        telemetry.addLine();

        PIDFCoefficients pidf = (currentMode == TuningMode.TURRET) ? turretPIDF : outtakePIDF;
        telemetry.addData("P", formatCoeff(pidf.p, Coefficient.P));
        telemetry.addData("I", formatCoeff(pidf.i, Coefficient.I));
        telemetry.addData("D", formatCoeff(pidf.d, Coefficient.D));
        telemetry.addData("F", formatCoeff(pidf.f, Coefficient.F));

        telemetry.addLine();

        if (currentMode == TuningMode.TURRET && turret != null)
        {
            telemetry.addData("Position", String.format("%.2f°", turret.getDistance()));
            telemetry.addData("At Target", turret.atSetPoint());
        }
        else if (currentMode == TuningMode.OUTTAKE && outtake != null)
        {
            telemetry.addData("RPM", String.format("%.0f / %.0f", outtake.getRPM(), outtake.getTargetRPM()));
            telemetry.addData("At Target", outtake.atSetPoint());
        }

        if (isTestRunning || errorHistory.size() > 0)
        {
            telemetry.addLine("--- Test Data ---");
            telemetry.addData("Test Running", isTestRunning);
            telemetry.addData("Peak Overshoot", String.format("%.2f", peakOvershoot));
            telemetry.addData("Settling Time", String.format("%.3fs", settlingTime));
            telemetry.addData("Steady-State Error", String.format("%.3f", steadyStateError));
            telemetry.addData("Samples", errorHistory.size());
        }

        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A/B: Turret/Outtake | DPAD: Select/Adjust");
        telemetry.addLine("X: Test | Y: Stop | START: Suggestions | BACK: Reset");

        telemetry.update();
    }

    private String formatCoeff(double value, Coefficient coeff)
    {
        String marker = (selectedCoeff == coeff) ? " <<" : "";
        return String.format("%.6f%s", value, marker);
    }

    @Override
    public void stop()
    {
        if (turret != null)
        {
            turret.stopMotor();
        }
        if (outtake != null)
        {
            outtake.stopMotor();
        }
    }
}
