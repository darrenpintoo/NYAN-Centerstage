package org.firstinspires.ftc.teamcode.utilities.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {

    private DcMotorEx encoder;
    private DcMotorSimple.Direction direction;
    public Encoder(DcMotorEx motor, DcMotorSimple.Direction direction) {
        this.encoder = motor;
        this.direction = direction;
    }

    public getTicks() {
        
    }


}
