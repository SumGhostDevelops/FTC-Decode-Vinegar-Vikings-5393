import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Autonomous { private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices
        public void init(HardwareMap hardwareMap) {
            frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
            backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
            frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
            backRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
}
