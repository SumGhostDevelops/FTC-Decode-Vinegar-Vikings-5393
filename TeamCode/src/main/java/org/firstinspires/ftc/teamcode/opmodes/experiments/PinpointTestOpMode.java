// Java
package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;

@TeleOp(name="Pinpoint Test", group="Experimental")
public class PinpointTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("info","Attempting to get Pinpoint");
        telemetry.update();

        Pinpoint pinpoint = null;
        try {
            pinpoint = hardwareMap.get(Pinpoint.class, RobotConstants.Odometry.Pinpoint.NAME);
        } catch (Exception e) {
            telemetry.addData("get() failed", e.getMessage());
            telemetry.update();
        }

        if (pinpoint == null) {
            telemetry.addData("status", "Pinpoint not found (null)");
            telemetry.update();
            waitForStart(); // allow viewing the message on driver station
            return;
        }

        telemetry.addData("status","Pinpoint instance acquired");
        try {
            telemetry.addData("deviceID", pinpoint.getDeviceID());
            telemetry.addData("version", pinpoint.getDeviceVersion());
            pinpoint.update();
            telemetry.addData("deviceStatus", pinpoint.getDeviceStatus().name());
            telemetry.addData("frequencyHz", String.format("%.1f", pinpoint.getFrequency()));
        } catch (Exception e) {
            telemetry.addData("read error", e.getMessage());
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try {
                pinpoint.update();
                telemetry.addData("status","OK");
                telemetry.addData("deviceStatus", pinpoint.getDeviceStatus().name());
                telemetry.addData("freqHz", String.format("%.1f", pinpoint.getFrequency()));
            } catch (Exception e) {
                telemetry.addData("runtime error", e.getMessage());
            }
            telemetry.update();
            sleep(500);
        }
    }
}
