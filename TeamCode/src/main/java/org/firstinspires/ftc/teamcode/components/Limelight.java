package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private LLResult llResult;
    private final int DEFAULT_PIPELINE = 0;

    public Limelight(Limelight3A limelight){
        this.limelight = limelight;
        limelight.pipelineSwitch(DEFAULT_PIPELINE);
    }


    public Limelight(Limelight3A limelight, int pipline){
        this.limelight = limelight;
        limelight.pipelineSwitch(pipline);
    }

    // This function is required to execute right when the OpMode starts (after init)
    public void startLimelight(){
        limelight.start();
    }

    public double getDx() {
        if (llResult != null && llResult.isValid()) return llResult.getTx();
        return 0;
    }

    public double getDy(){
        if (llResult != null && llResult.isValid()) return llResult.getTx();
        return 0;
    }

    public double getDa(){
        if (llResult != null && llResult.isValid()) return llResult.getTa();
        return 0;
    }

    private void operate() {
        llResult = limelight.getLatestResult();

    }
}
