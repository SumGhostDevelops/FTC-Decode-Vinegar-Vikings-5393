// java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

public class Outtake extends SubsystemBase {

    private final MotorExPlusGroup motor;

    // Accept a single MotorExPlus (RobotContext should provide this)
    public Outtake(MotorExPlusGroup motor) {
        this.motor = motor;
    }

    // Open-loop power control (keeps previous name)
    public void in(double power) {
        motor.set(power);
    }

    // Closed-loop: set a target velocity (RPM)
    public void setRPM(double rpm) {
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setRPM(rpm);
    }

    // Read current motor velocity (RPM)
    public double getVelocity() {
        return motor.getRPM();
    }

    public double getAcceleration(){
        double total=0;
        double count=0;
        for(double x:motor.getAccelerations()){
            total+=x;
            count+=1;
        };
        return total/count;
    }

    public void stop() {
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(0.0);
    }
}
