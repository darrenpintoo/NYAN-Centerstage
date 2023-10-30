package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {

    private DcMotorEx encoder;
    private int multiplier = 1;
    public Encoder(DcMotorEx motor, int multiplier) {
        this.encoder = motor;
        this.multiplier = multiplier;
    }

    public int getTicks() {
        return this.encoder.getCurrentPosition() * multiplier;
    }


}
