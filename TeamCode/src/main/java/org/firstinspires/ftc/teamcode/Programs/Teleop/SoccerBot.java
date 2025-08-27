package org.firstinspires.ftc.teamcode.Programs.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.GlobalData;

@TeleOp(name="SoccerTeleop", group="!")  // Registers this as a TeleOp mode named "RedTeleop"
public class SoccerBot extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private static final double SPEED_MODIFIER = 0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()){
            double spdX = gamepad1.left_stick_x * SPEED_MODIFIER;
            double spdY = -gamepad1.left_stick_y * SPEED_MODIFIER;
            double spdTurn = gamepad1.right_stick_x * SPEED_MODIFIER;

            leftFront.setPower((spdY + spdX + spdTurn));
            rightFront.setPower((spdY - spdX - spdTurn));
            leftBack.setPower((spdY - spdX + spdTurn));
            rightBack.setPower((spdY + spdX - spdTurn));
        }
    }
}
