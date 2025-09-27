import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;

// This is initialization

public class ControlHub
{


    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public ElapsedTime timer;
    MecanumDrive drive;
    public CRServo BasicServo;
    public Servo RegularServo;
    public DcMotor leftWheel;

    public DcMotor rightWheel;



    public void init(HardwareMap map, Pose2d initialPose) {
        leftFront = map.get(DcMotor.class, "leftFront");
        rightFront = map.get(DcMotor.class, "rightFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightBack = map.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //This is one of the motors to move the arm up and down
        //leftWheel = map.get(DcMotor.class, "leftWheel");
        //This is one of the motors to move the arm up and down
        //rightWheel = map.get(DcMotor.class, "rightWheel");

        //camera=map.get(WebcamName.class,"camera");
        drive=new MecanumDrive(map,initialPose);
    }
}
