package org.firstinspires.ftc.teamcode.vision.simulatortests;

public enum PlacementPosition {
    LEFT(1),
    CENTER(2),
    RIGHT(3);

    public final int position;

    PlacementPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return this.position;
    }
}
