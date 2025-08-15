package org.firstinspires.ftc.teamcode.programs.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.GlobalData;
import org.firstinspires.ftc.teamcode.components.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Payload;

@Autonomous(name = "Blue Autonomous", group = "AutonomousMain")
public class AutonomousBlue extends LinearOpMode {

    // --- Pose constants ---
    private static final Pose2d START_POSE = new Pose2d(0, -60, Math.toRadians(270));
    private static final Pose2d SCORE_POSE = new Pose2d(0, -32, Math.toRadians(270));

    private Arm arm;
    private Payload payload;
    private MecanumDrive drive;

    private Action traj;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        initializeTrajectories();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                moveArmTo(Arm.PREPARE_SPECIMEN),
                                traj
                        ),
                        moveArmTo(Arm.SCORE_SPECIMEN),
                        unloadSample()
                ),
                operate()
        ));
    }


    private Action buildStrafeToLinearHeading(Pose2d start, Pose2d end) {
        return drive.actionBuilder(start)
                .strafeToLinearHeading(end.position, end.heading)
                .build();
    }

    private void initializeRobot(){
        arm = new Arm(
                hardwareMap.get(DcMotor.class, "leftArm"),
                hardwareMap.get(DcMotor.class, "rightArm"),
                GlobalData.OpModeType.AUTONOMOUS
        );

        payload = new Payload(
                hardwareMap.get(RevBlinkinLedDriver.class, "leds"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor"),
                hardwareMap.get(CRServo.class, "rightEndEffector"),
                hardwareMap.get(CRServo.class, "leftEndEffector"),
                GlobalData.Alliance.BLUE
        );


        drive = new MecanumDrive(hardwareMap, START_POSE);
    }

    private void initializeTrajectories(){
        traj = buildStrafeToLinearHeading(START_POSE, SCORE_POSE);
    }

    private class Operate implements Action{

        private void operateTelemetry(){
            telemetry.update();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.operate();
            arm.operate();
            operateTelemetry();
            return true;
        }
    }
    private Action operate(){
        return new Operate();
    }

    private class MoveArmTo implements Action{

        private double angle;

        public MoveArmTo(double angle){
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetAngle(angle);
            return !arm.arrivedTargetAngle() && !arm.armNotMoving();
        }
    }
    private Action moveArmTo(double angle){
        return new MoveArmTo(angle);
    }

    private class UnloadSample implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.unload();
            return !payload.isSampleDetected();
        }
    }
    private Action unloadSample(){
        return new UnloadSample();
    }
}
