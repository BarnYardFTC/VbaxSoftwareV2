package org.firstinspires.ftc.teamcode.components;

import android.widget.EdgeEffect;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class EndEffector {
    private CRServo rightServo, leftServo;
    public EndEffector(CRServo rightServo, CRServo leftServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void hold() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
    public void intake() {
        leftServo.setPower(1);
        rightServo.setPower(1);
    }
    public void unload() {
        leftServo.setPower(-1);
        rightServo.setPower(-1);
    }
}
