package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Template Auto")
public class TemplateAuto extends LinearOpMode {


    OpenCvCamera frontCamera;
    String cameraName = "Webcam 1";


    @Override
    public void runOpMode() {
        MotionProfileLocalizerLineDrive drive = new MotionProfileLocalizerLineDrive(this, telemetry);

        RobotEx robot = RobotEx.getInstance();


        waitForStart();
        while (true) {
            drive.turnToAngle(Math.toRadians(90));
            robot.pause(1);
            drive.turnToAngle(Math.toRadians(-90));
        }
    }
}
