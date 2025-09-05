package org.firstinspires.ftc.teamcode.programs.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.GlobalData;
import org.firstinspires.ftc.teamcode.components.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Payload;

public class AutoController {

    private Arm arm;
    private MecanumDrive drive;
    private Payload payload;
    private GlobalData.Alliance allianceColor;

    public AutoController(OpMode opMode, GlobalData.Alliance alliance, Pose2d startPos){
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.arm = new Arm(
                hardwareMap.get(DcMotorEx.class, "leftArm"),
                hardwareMap.get(DcMotorEx.class, "rightArm"),
                GlobalData.OpModeType.TELEOP
        );

        this.payload = new Payload(
                hardwareMap.get(RevBlinkinLedDriver.class, "leds"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor"),
                hardwareMap.get(CRServo.class, "rightEndEffector"),
                hardwareMap.get(CRServo.class, "leftEndEffector"),
                alliance
        );

        this.drive = new MecanumDrive(hardwareMap, startPos);
        this.allianceColor = alliance;
    }

    public Action operate(){
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

    public Action moveArmTo(double angle){
        return new MoveArmTo(angle);
    }
    private class MoveArmTo implements Action{
        private double angle;
        public MoveArmTo(double angle){
            this.angle = angle;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetAngle(angle);
            return !arm.hasArrived();
        }
    }

    public Action unloadSample(){
        return new UnloadSample();
    }
    private class UnloadSample implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            payload.unload();
            return payload.sampleFullyIn;
        }
    }

    public Action buildStrafeToLinearHeading(MecanumDrive drive, Pose2d start, Pose2d end){
        return drive.actionBuilder(start).strafeToLinearHeading(end.position, end.heading).build();

    }

}
