import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Autonomous { private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices
        public void init(HardwareMap hardwareMap) {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
            backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
}
