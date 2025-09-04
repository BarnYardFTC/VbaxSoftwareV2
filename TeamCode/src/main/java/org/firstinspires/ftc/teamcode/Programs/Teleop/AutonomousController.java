package org.firstinspires.ftc.teamcode.Programs.Teleop;

import android.graphics.Path;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.onbotjava.handlers.file.TemplateFile;
import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Components.GlobalData;

public class AutonomousController{
    private Arm arm;
    private MecanumDrive mecanumDrive;
    private GamepadEx gamepadEx1;
    private GamepadEx getGamepadEx2;


    public AutonomousController(OpMode opMode, GlobalData.Alliance alliance){
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.arm = new Arm(
            hardwareMap.get(DcMotorEx.class ,"leftArm"),
            hardwareMap.get(DcMotorEx.class, "rightArm"),
            GlobalData.OpmodeType.AUTONOMOUS
        );
//        this.mecanumDrive = new MecanumDrive(hardwareMap, );






    }
}



