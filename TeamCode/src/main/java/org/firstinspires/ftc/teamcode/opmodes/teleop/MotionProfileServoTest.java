package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;

@TeleOp
public class MotionProfileServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftRotationServo = hardwareMap.get(Servo.class, "leftRotationServo");
        Servo rightRotationServo = hardwareMap.get(Servo.class, "rightRotationServo");

        waitForStart();

        leftRotationServo.setPosition(0.2);
        rightRotationServo.setPosition(0.2);

        sleep(1000);

        MotionProfile mp = new MotionProfile(
                0.2,
                0.75,
                1,
                1
        );

        ElapsedTime et = new ElapsedTime();

        while (et.seconds() < mp.getDuration()) {
            double position = mp.getPositionFromTime(et.seconds());
            leftRotationServo.setPosition(position);
            rightRotationServo.setPosition(position);
        }
        mp = new MotionProfile(
                0.75,
                0.2,
                1,
                1
        );
        sleep(1000);
        et.reset();
        while (et.seconds() < mp.getDuration()) {
            double position = mp.getPositionFromTime(et.seconds());
            leftRotationServo.setPosition(position);
            rightRotationServo.setPosition(position);
        }
        while (opModeIsActive()) {
            leftRotationServo.setPosition(0.2);
            rightRotationServo.setPosition(0.2);
        }
    }
}
