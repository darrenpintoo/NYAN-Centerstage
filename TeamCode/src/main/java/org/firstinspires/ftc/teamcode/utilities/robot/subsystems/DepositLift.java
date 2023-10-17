package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

public class DepositLift implements Subsystem{

    double power;

    public DcMotorEx backLiftMotor;
    public DcMotorEx frontLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.backLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backDepositMotor");
        this.frontLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontDepositMotor");

        this.liftMotors = new MotorGroup<>(backLiftMotor, frontLiftMotor);
    }

    @Override
    public void onOpmodeStarted() {
        this.backLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveLiftFromGamepad(double power) {
        this.power = power;
    }
    @Override
    public void onCyclePassed() {
        this.liftMotors.setPower(power);

        power = 0;
    }
}