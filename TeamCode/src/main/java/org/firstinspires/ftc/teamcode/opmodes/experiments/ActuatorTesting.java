package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Actuator Testing", group = "Experiments")
public class ActuatorTesting extends Base
{
    // Inner class to hold actuator information
    private static class ActuatorEntry
    {
        final String name;
        final DcMotorEx motor;
        final CRServo servo;
        final boolean isAvailable;

        // Constructor for motor actuators
        ActuatorEntry(String name, DcMotorEx motor)
        {
            this.name = name;
            this.motor = motor;
            this.servo = null;
            this.isAvailable = motor != null;
        }

        // Constructor for servo actuators
        ActuatorEntry(String name, CRServo servo)
        {
            this.name = name;
            this.motor = null;
            this.servo = servo;
            this.isAvailable = servo != null;
        }

        // Constructor for "None" safety option
        ActuatorEntry()
        {
            this.name = "None (Safety)";
            this.motor = null;
            this.servo = null;
            this.isAvailable = true; // Always selectable
        }

        void setPower(double power)
        {
            if (!isAvailable) return;
            if (motor != null)
            {
                motor.setPower(power);
            }
            else if (servo != null)
            {
                servo.setPower(power);
            }
            // "None" option does nothing
        }

        void stop()
        {
            setPower(0);
        }
    }

    private List<ActuatorEntry> actuators;
    private int hoveredIndex = 0;      // Currently hovered actuator in menu
    private int selectedIndex = 0;     // Currently selected actuator (0 = None by default)

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }

    @Override
    protected void initSystems()
    {
        super.initSystems();
        buildActuatorList();
    }

    private void buildActuatorList()
    {
        actuators = new ArrayList<>();

        // Index 0: Safety "None" option (default selected)
        actuators.add(new ActuatorEntry());

        // Drive motors
        actuators.add(new ActuatorEntry("Front Left Drive", robot.hw.frontLeft));
        actuators.add(new ActuatorEntry("Front Right Drive", robot.hw.frontRight));
        actuators.add(new ActuatorEntry("Back Left Drive", robot.hw.backLeft));
        actuators.add(new ActuatorEntry("Back Right Drive", robot.hw.backRight));

        // Mechanism motors
        actuators.add(new ActuatorEntry("Outtake Left Motor", robot.hw.outtakeLeftMotor));
        actuators.add(new ActuatorEntry("Outtake Right Motor", robot.hw.outtakeRightMotor));
        actuators.add(new ActuatorEntry("Turret Motor", robot.hw.turretMotor));
        actuators.add(new ActuatorEntry("Intake Motor", robot.hw.intakeMotor));

        // Servos
        actuators.add(new ActuatorEntry("Transfer Servo", robot.hw.transferServo));

        // Reset indices
        hoveredIndex = 0;
        selectedIndex = 0;
    }

    @Override
    protected void bindKeys()
    {
        // DPAD UP - Navigate up in menu
        input.bind(
                () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                this::navigateUp
        );

        // DPAD DOWN - Navigate down in menu
        input.bind(
                () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                this::navigateDown
        );

        // A Button - Select current hovered actuator
        input.bind(
                () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.A),
                this::selectActuator
        );

        // B Button - Deselect (return to None/Safety)
        input.bind(
                () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.B),
                () -> {
                    // Stop current actuator before deselecting
                    actuators.get(selectedIndex).stop();
                    selectedIndex = 0;
                    hoveredIndex = 0;
                }
        );
    }

    private void navigateUp()
    {
        // Stop current actuator when navigating
        actuators.get(selectedIndex).stop();

        int startIndex = hoveredIndex;
        do
        {
            hoveredIndex--;
            if (hoveredIndex < 0)
            {
                hoveredIndex = actuators.size() - 1;
            }
            // Skip unavailable actuators, but allow wrapping
            if (hoveredIndex == startIndex) break; // Prevent infinite loop
        }
        while (!actuators.get(hoveredIndex).isAvailable);
    }

    private void navigateDown()
    {
        // Stop current actuator when navigating
        actuators.get(selectedIndex).stop();

        int startIndex = hoveredIndex;
        do
        {
            hoveredIndex++;
            if (hoveredIndex >= actuators.size())
            {
                hoveredIndex = 0;
            }
            // Skip unavailable actuators, but allow wrapping
            if (hoveredIndex == startIndex) break; // Prevent infinite loop
        }
        while (!actuators.get(hoveredIndex).isAvailable);
    }

    private void selectActuator()
    {
        ActuatorEntry entry = actuators.get(hoveredIndex);
        if (entry.isAvailable)
        {
            // Stop previous actuator
            actuators.get(selectedIndex).stop();
            selectedIndex = hoveredIndex;
        }
    }

    @Override
    protected void run() throws InterruptedException
    {
        // Update gamepads for button detection
        robot.gamepads.gamepad1.readButtons();

        // Display header
        telemetry.addLine("===== ACTUATOR TESTING =====");
        telemetry.addLine("DPAD: Navigate | A: Select | B: Deselect");
        telemetry.addLine("Left Stick Y: Control Power");
        telemetry.addLine("");

        // Display currently selected actuator
        ActuatorEntry selected = actuators.get(selectedIndex);
        telemetry.addData("SELECTED", selected.name);
        telemetry.addLine("");

        // Display menu
        telemetry.addLine("----- Actuator Menu -----");
        for (int i = 0; i < actuators.size(); i++)
        {
            ActuatorEntry entry = actuators.get(i);
            String prefix = "";

            // Show hover indicator
            if (i == hoveredIndex)
            {
                prefix = "> ";
            }
            else
            {
                prefix = "  ";
            }

            // Show selection indicator
            if (i == selectedIndex)
            {
                prefix += "[*] ";
            }
            else
            {
                prefix += "[ ] ";
            }

            // Show availability
            if (!entry.isAvailable)
            {
                telemetry.addLine(prefix + entry.name + " (UNAVAILABLE)");
            }
            else
            {
                telemetry.addLine(prefix + entry.name);
            }
        }

        // Get joystick input and apply power to selected actuator
        double power = -gamepad1.left_stick_y; // Inverted so up = positive

        // Use right stick if it has greater magnitude
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.left_stick_y))
        {
            power = -gamepad1.right_stick_y;
        }

        // Apply power to selected actuator
        selected.setPower(power);

        // Display power info
        telemetry.addLine("");
        telemetry.addData("Applied Power", String.format("%.2f", power));

        telemetry.update();
    }
}
