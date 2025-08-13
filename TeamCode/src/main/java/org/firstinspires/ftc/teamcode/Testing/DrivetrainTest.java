package org.firstinspires.ftc.teamcode.Testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DrivetrainTest", group="test")
public class DrivetrainTest extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private double speedX, speedY, speedTurn;
    private SparkFunOTOS otos;
    private double speedModifier;
    private final double SLOW_SPEED = 0.3;
    private final double FAST_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            determineSpeed();
            translateSpeedToPower();
            resetHeading();
        }
    }

    private DcMotor initMotor(String deviceName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private void initData() {
        speedX = 0;
        speedY = 0;
        speedTurn = 0;
        activateFastMode();
    }

    private void initOtos() {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
    }

    private void initialize() {
        leftFront = initMotor("leftFront", DcMotorSimple.Direction.REVERSE);
        rightFront = initMotor("rightFront", DcMotorSimple.Direction.FORWARD);
        leftBack = initMotor("leftBack", DcMotorSimple.Direction.REVERSE);
        rightBack = initMotor("rightBack", DcMotorSimple.Direction.FORWARD);
        initData();
        initOtos();
    }

    private double getHeading() {
        return otos.getPosition().h;
    }

    private void adjustSpeed() {
        double adjustedX = speedX * Math.cos(getHeading()) - speedY * Math.sin(getHeading());
        double adjustedY = speedX * Math.sin(getHeading()) + speedY * Math.cos(getHeading());
        speedX = adjustedX;
        speedY = adjustedY;
    }

    private void determineSpeed() {

        if (gamepad1.a) activateSlowMode();
        else if (gamepad1.b) activateFastMode();

        speedX = -gamepad1.left_stick_x;
        speedY = -gamepad1.left_stick_y;
        speedTurn = gamepad1.right_stick_x;
        adjustSpeed();
    }

    private void translateSpeedToPower() {
        double lf = speedY + speedX + speedTurn;
        double lb = speedY - speedX + speedTurn;
        double rf = speedY - speedX - speedTurn;
        double rb = speedY + speedX - speedTurn;

        double maxPower = Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb)));

        if (maxPower > 1.0) {
            lf /= maxPower;
            lb /= maxPower;
            rf /= maxPower;
            rb /= maxPower;
        }

        lf *= speedModifier;
        lb *= speedModifier;
        rf *= speedModifier;
        rb *= speedModifier;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }


    private void resetHeading() {
        if (gamepad1.x) {
            otos.resetTracking();
        }
    }

    private void activateSlowMode() {
        speedModifier = SLOW_SPEED;
    }

    private void activateFastMode() {
        speedModifier = FAST_SPEED;
    }
}
