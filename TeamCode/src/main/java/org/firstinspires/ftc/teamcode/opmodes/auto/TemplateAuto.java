package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Template Auto")
public class TemplateAuto extends LinearOpMode {


    OpenCvCamera frontCamera;
    String cameraName = "Webcam 1";

    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        waitForStart();

        // Notify subsystems before loop
        robot.postInit();
        OneWheelOdometryDrive drive = new OneWheelOdometryDrive(this, telemetry);


        waitForStart();

        while (!isStopRequested()) {
            drive.driveForward(robot.drivetrain.rightBackMotor, 30, Math.toRadians(0));
            drive.turnToAngle(Math.toRadians(-180));
            drive.driveForward(robot.drivetrain.rightBackMotor, 30, Math.toRadians(-180));
            drive.turnToAngle(Math.toRadians(0));

        }

    }
}
