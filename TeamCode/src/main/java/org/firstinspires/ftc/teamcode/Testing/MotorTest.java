package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
How to create a Teleop OpMode?
1. @Teleop(name="{name}", group="{group}")
2. add "extends LinearOpMode" and the runOpMode() method
3. add class variables
4. add waitForStart() and while(opModeIsActive())
5. initialize the robot
6. run the opMode
 */

@TeleOp(name="MotorTest", group="test")
public class MotorTest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(0.4);
            telemetry.addData("encoder position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
    private void initialize() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
