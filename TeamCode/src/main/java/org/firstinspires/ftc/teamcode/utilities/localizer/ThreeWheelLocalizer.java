package org.firstinspires.ftc.teamcode.utilities.localizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.Encoder;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;

public class ThreeWheelLocalizer {

    Telemetry telemetry;

    Encoder leftParallel;
    Encoder rightParallel;
    Encoder backPerpendicular;

    public ThreeWheelLocalizer(Encoder lp, Encoder rp, Encoder bp, Telemetry telemetry) {
        this.leftParallel = lp;
        this.rightParallel = rp;
        this.backPerpendicular = bp;
        this.telemetry = telemetry;
    }

    public void updatePose() {
        this.telemetry.addData("lp: ", leftParallel.getTicks());
        this.telemetry.addData("rp: ", rightParallel.getTicks());
        this.telemetry.addData("bp: ", backPerpendicular.getTicks());

    }
}
