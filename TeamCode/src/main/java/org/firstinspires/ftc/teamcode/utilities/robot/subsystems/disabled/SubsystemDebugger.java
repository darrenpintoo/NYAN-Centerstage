package org.firstinspires.ftc.teamcode.utilities.robot.subsystems.disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;

public abstract class SubsystemDebugger implements Subsystem {

    Telemetry telemetry;

    protected SubsystemDebugger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    protected Telemetry getTelemetryInstance() {
        return this.telemetry;
    }
}
