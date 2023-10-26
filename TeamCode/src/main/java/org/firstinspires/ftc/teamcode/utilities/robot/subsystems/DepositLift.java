package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class DepositLift implements Subsystem{

    public enum LiftStates {
        LEVEL0,
        LEVEL1,
        LEVEL2,
        LEVEL3
    }
    double power;

    public DcMotorEx backLiftMotor;
    public DcMotorEx frontLiftMotor;

    private MotorGroup<DcMotorEx> liftMotors;

    private LiftStates currentTargetState = LiftStates.LEVEL0;

    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.15;
    public static int targetPosition;
    private GeneralPIDController controller = new GeneralPIDController(0, 0, 0, 0);

    private Telemetry t;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.backLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backDepositMotor");
        this.frontLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontDepositMotor");

        this.liftMotors = new MotorGroup<>(backLiftMotor, frontLiftMotor);

        t = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        this.backLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveLiftFromGamepad(double power) {
        this.power = power;
    }
    @Override
    public void onCyclePassed() {

        controller.updateCoefficients(kP, kI, kD, kF);

        int targetPosition = this.getTargetPositionFromState(this.currentTargetState);

        if (power == 0) {
            power = MathHelper.clamp(controller.getOutputFromError(targetPosition, this.frontLiftMotor.getCurrentPosition()), -0.75, 0.75);
        }

        t.addData("Power:", power);
        t.addData("Current Position: ", this.frontLiftMotor.getCurrentPosition());
        t.addData("Target Position: ", targetPosition);
        this.liftMotors.setPower(power);

        power = 0;
    }

    public int getTargetPositionFromState(LiftStates state) {
        switch (state) {
            case LEVEL0:
                return 0;
            case LEVEL1:
                return 400;
            case LEVEL2:
                return 700;
            case LEVEL3:
                return 1200;
            default:
                return 1000;
        }
    }

    public void setTargetState(LiftStates state) {
        this.currentTargetState = state;
    }
}