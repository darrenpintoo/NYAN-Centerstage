package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BaseEncoder {

    private DcMotorEx encoder;
    private int multiplier = 1;
    public BaseEncoder(DcMotorEx motor, int multiplier) {
        this.encoder = motor;
        this.multiplier = multiplier;
    }

    public int getTicks() {
        return this.encoder.getCurrentPosition() * multiplier;
    }


}
