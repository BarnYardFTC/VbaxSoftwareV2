package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.GlobalData;
import org.firstinspires.ftc.teamcode.Components.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Payload;

import java.util.Optional;

public class AutonomousController {

    private Arm arm;
    private Payload payload;
    private MecanumDrive drive;

    private final GlobalData.Alliance alliance;

    private OpMode opMode;

    public AutonomousController(OpMode opMode, Pose2d startPose, GlobalData.Alliance alliance) {
        this.opMode = opMode;
        this.alliance = alliance;
        initializeRobot(opMode.hardwareMap, startPose);
    }

    private void initializeRobot(HardwareMap hardwareMap, Pose2d startPose) {
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
                alliance
        );

        drive = new MecanumDrive(hardwareMap, startPose);
    }

    // --------- Common Actions ---------

    public Action operate() {
        return new Operate();
    }

    private class Operate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.operate();
            arm.operate();
            return true;
        }
    }

    public Action moveArmTo(double angle) {
        return new MoveArmTo(angle);
    }

    private class MoveArmTo implements Action {
        private final double angle;

        public MoveArmTo(double angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetAngle(angle);
            return !arm.arrivedTargetAngle() && !arm.armNotMoving();
        }
    }

    public Action unloadSample() {
        return new UnloadSample();
    }

    private class UnloadSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.unload();
            return payload.isSampleDetected();
        }
    }

    // --------- Common Helpers ---------
    public Action buildStrafeToLinearHeading(Pose2d start, Pose2d end) {
        return drive.actionBuilder(start)
                .strafeToLinearHeading(end.position, end.heading)
                .build();
    }
}
