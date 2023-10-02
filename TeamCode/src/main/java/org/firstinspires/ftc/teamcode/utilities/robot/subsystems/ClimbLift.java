package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

public class ClimbLift implements Subsystem {

    double power;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftLiftMotor");
        this.rightLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightLiftMotor");
    }

    @Override
    public void onOpmodeStarted() {
        this.leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setLiftPower(double power) {
        this.power = power;
    }
    @Override
    public void onCyclePassed() {
        this.leftLiftMotor.setPower(power);
        this.rightLiftMotor.setPower(power);

        power = 0;
    }
}
