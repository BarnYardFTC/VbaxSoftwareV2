package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Limelight {

    private Limelight3A limelight;
    private LLResult llResult;
    private double Dx, Dy, Dt;
    private final int PIPELINE = 7;

    public Limelight(Limelight3A limelight){
        this.limelight = limelight;
        Dx = 0;
        Dy = 0;
        Dt = 0;
        limelight.pipelineSwitch(PIPELINE);
    }
    // This function is required to execute right when the OpMode starts (after init)
    public void start(){
        limelight.start();
    }

    public double getDx() {
        return Dx;
    }

    public double getDy(){
        return Dy;
    }

    public double getDt(){
        return Dt;
    }

    private void calculateD(LLResultTypes.FiducialResult fr){
        Dx = fr.getCameraPoseTargetSpace().getPosition().x;
        Dy = fr.getCameraPoseTargetSpace().getPosition().z;
        Dt = fr.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.DEGREES);
    }

    public void operate() {
        llResult = limelight.getLatestResult();
        if (llResult.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                calculateD(fr);
            }
        }
        Dx = 0; Dy = -0.22; Dt = 0;

    }
}
