package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotContext;

@TeleOp
public class Servos extends OpMode {
   public CRServo servo;
   public RobotContext robotContext;
    @Override
    public void init(){

    }

    public void loop(){

        while(robotContext.gamepad.right_trigger > 0.25) { // Shoot
          servo.setPower(1);
        }
        servo.setPower(0);
    }

}

