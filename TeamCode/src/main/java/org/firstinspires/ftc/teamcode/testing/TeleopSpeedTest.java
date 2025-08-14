package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.teleop.TeleopController;

@TeleOp(name="TeleopSpeedTest", group="Testing")
public class TeleopSpeedTest extends LinearOpMode {

    private TeleopController teleopController;
    private boolean isAction;
    private int loopCount;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

//        teleopController = new TeleopController(
//                hardwareMap,
//                new GamepadEx(gamepad1),
//                new GamepadEx(gamepad2),
//                GlobalData.Alliance.BLUE
//        );

        isAction = false;

        telemetry.addLine("Press A on gamepad1 to test with Action");
        telemetry.addLine("Don't press A to test without Action");
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.a) isAction = true;
        }

        waitForStart();
        telemetry.clearAll();

        loopCount = 0;

        if (isAction) {
            telemetry.addLine("Running with Action...");
            telemetry.update();

            final ElapsedTime timer = new ElapsedTime();


            timer.reset();
//            Actions.runBlocking(countAction);

        } else {
            telemetry.addLine("Running without Action...");
            telemetry.update();

            timer = new ElapsedTime();
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 1.0) {
                loopCount++;
            }
        }

        telemetry.clearAll();
        telemetry.addData("Test", isAction ? "With Action" : "Without Action");
        telemetry.addData("Loops in 1 second", loopCount);
        telemetry.update();

        sleep(5000);
    }


//    public class CountAction implements Action{
//
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            loopCount++;
//            return ;
//        }
//    }
}
