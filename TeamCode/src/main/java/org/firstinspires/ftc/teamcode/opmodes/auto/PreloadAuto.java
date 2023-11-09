package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="Preload Auto")
public class PreloadAuto extends LinearOpMode {


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

        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        waitForStart();

        /*
        drive.driveForward(robot.drivetrain.rightBackMotor, 20, Math.toRadians(0));
        drive.turnToAngle(Math.toRadians(-90));
        drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(-90));
        drive.turnToAngle(Math.toRadians(0));
        drive.driveForward(robot.drivetrain.rightBackMotor, 30, Math.toRadians(0));
         */
        robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
        drive.driveForward(robot.drivetrain.rightBackMotor, 50, Math.toRadians(0));
        robot.pause(1);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);

    }
}
