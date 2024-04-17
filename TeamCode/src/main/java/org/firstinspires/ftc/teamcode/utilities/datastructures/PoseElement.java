package org.firstinspires.ftc.teamcode.utilities.datastructures;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class PoseElement {

    private Pose element;

    private double timestamp;

    public PoseElement(Pose element, double timestamp) {
        this.element = element;
        this.timestamp = timestamp;
    }

    public Pose getElement() {
        return element;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
