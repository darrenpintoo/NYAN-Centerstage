package org.firstinspires.ftc.teamcode.utilities.robot;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.localizer.TwoWheelLocalizerRoadrunner;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.ClimbLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;


import java.util.List;

/*
Robot Configuration
Control Hub
    Motors:
        0 - rightBackMotor
        1 - rightFrontMotor
        2 - leftFrontMotor
        3 - leftBackMotor
    Servos:
        0 - intakeClaw
        1 - boxOpen
        2 - rightBox
        3 - leftBox
        4 - leftIntakeServo
        5 - rightIntakeServo
    DPIO
        0/1 - breakBeams
        2/3 - leftProximity
        4/5 - centerProximity
        6/7 - rightProximity
Expansion Hub
    Motors:
    Servos:

 */
public class RobotEx {
    private static RobotEx robotInstance = null;

    List<LynxModule> allHubs;

    public InternalIMU internalIMU = InternalIMU.getInstance();

    public Drivetrain drivetrain = new Drivetrain();
    public ClimbLift climbLift = new ClimbLift();
    public DepositLift depositLift = new DepositLift();
    public Intake intake = new Intake();
    public VoltageSensor voltageSensor;

    public PlaneLauncher planeLauncher = new PlaneLauncher();
    public Camera backCamera = new Camera();
    private final ElapsedTime frameTimer = new ElapsedTime();

    private final Subsystem[] robotSubsystems = new Subsystem[]{
            drivetrain,
            climbLift,
            intake,
            depositLift,
            planeLauncher,
            backCamera
    };

    Telemetry telemetry;

    public TwoWheelLocalizer localizer;
    public HardwareMap hardwareMap;

    public LinearOpMode opMode;

    public boolean stopRequested = false;
    public double runTime = 0;

    private double voltageCompensator = 12;
    private double frames = 0;
    private double currentFrames = 0;
    private double lastTime = 0;


    private RobotEx() {
        if (RobotEx.robotInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static RobotEx getInstance() {
        if (RobotEx.robotInstance == null) {
            RobotEx.robotInstance = new RobotEx();
        }

        return RobotEx.robotInstance;
    }



    public void init(LinearOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = telemetry;

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltageCompensator = this.voltageSensor.getVoltage();

        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onInit(hardwareMap, telemetry);
        }

        internalIMU.onInit(hardwareMap, telemetry);

        this.localizer = new TwoWheelLocalizer(
                new BaseEncoder(this.drivetrain.rightBackMotor, -1), // 0 LEFT // Swapped w/ 3
                new BaseEncoder(this.drivetrain.leftFrontMotor,  1), // 2 // Changed sign
                internalIMU,
                telemetry
        );

        telemetry.update();
    }

    public void init(LinearOpMode opMode) {
        this.init(opMode, opMode.telemetry);
    }

    public void postInit() {

        stopRequested = opMode.isStopRequested();

        if (stopRequested) return;

        this.allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (Subsystem subsystem : this.robotSubsystems) {
            subsystem.onOpmodeStarted();
        }

        internalIMU.onOpmodeStarted();
        internalIMU.stopAngularVelocityTracking();
    }

    @SuppressLint("")
    public double update() {

        stopRequested = opMode.isStopRequested();

        if (stopRequested) return 0;

        runTime = opMode.getRuntime();

        if (Math.floor(runTime) != lastTime) {
            frames = currentFrames;
            currentFrames = 0;
            lastTime = Math.floor(runTime);
        }

        currentFrames += 1;

        ElapsedTime log = new ElapsedTime();

        log.reset();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        localizer.updatePose();

        for (Subsystem subsystem : robotSubsystems) {
            subsystem.onCyclePassed();
        }


        telemetry.addData("Parallel Encoder 1 Ticks: ", drivetrain.rightBackMotor.getCurrentPosition());
        telemetry.addData("Parallel Encoder 2 Ticks: ", drivetrain.leftBackMotor.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder 1 Ticks: ", drivetrain.leftFrontMotor.getCurrentPosition());

        /*
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();


        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        */
        // drawRobot(fieldOverlay, currentPose);



        telemetry.addLine("Refresh Rate: " + frames + " hz");
        telemetry.addData("Run time: ", runTime);

        telemetry.update();

        double frameTime = frameTimer.milliseconds();
        frameTimer.reset();

        return frameTime;
    }

    public void pause(double seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.seconds() < seconds && !stopRequested) {
            update();
        }
    }

    public void persistData() {
        PersistentData.startPose = this.localizer.getPose();
    }

    public double getVoltage() {
        return this.voltageSensor.getVoltage();
    }

    public double getPowerMultiple() {
        return 12 / this.voltageCompensator;
    }

    /*
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }


    public void clearPersistData() {
        PersistentData.startPose = new Pose2d();
    }*/

    public void destroy() {
        RobotEx.robotInstance = null;
    }

}