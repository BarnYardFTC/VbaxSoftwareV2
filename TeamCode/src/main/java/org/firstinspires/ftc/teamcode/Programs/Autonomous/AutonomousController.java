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
import org.firstinspires.ftc.teamcode.Components.EndEffector;
import org.firstinspires.ftc.teamcode.Components.GlobalData;
import org.firstinspires.ftc.teamcode.Components.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Payload;

public class AutonomousController {
    private MecanumDrive mecanumDrive;
    private Arm arm;
    private Payload payload;
    private OpMode opMode;
    public AutonomousController(OpMode opMode, GlobalData.Alliance alliance, Pose2d pose2d) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.arm = new Arm(
                hardwareMap.get(DcMotor.class, "leftArm"),
                hardwareMap.get(DcMotor.class, "rightArm"), GlobalData.OpmodeType.TELEOP);
        this.payload = new Payload(
                hardwareMap.get(CRServo.class, "leftServo"),
                hardwareMap.get(CRServo.class, "rightServo"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor"),
                alliance,
                hardwareMap.get(RevBlinkinLedDriver.class, "leds"));
        this.mecanumDrive = new MecanumDrive(hardwareMap, pose2d);
    }
    public Action buildStrafeToLinearHeading(Pose2d start, Pose2d end){
        return mecanumDrive
                .actionBuilder(start)
                .strafeToLinearHeading(end.position, end.heading)
                .build();
    }
    private class Operate implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.operate();
            payload.operate();
            return true;
        }
    }
    public Action operate() {
        return new Operate();
    }
    private class UnloadSample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.unload();
            if (payload.isSampleIn()) return true;
            else return false;
        }
    }
    public Action unloadSample() {
        return new UnloadSample();
    }
    private class MoveArmTo implements Action {
        public MoveArmTo(double targetAngle) {
            arm.setTargetAngle(targetAngle);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !arm.hasArmArrived();
        }
    }
    public Action moveArmTo(double targetAnle) {
        return new MoveArmTo(targetAnle);
    }

}
