package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class LimelightTesting extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.
         */
        limelight.start();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> tagResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult cr : tagResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getRobotPoseFieldSpace());
            }
            telemetry.update();
        }
    }
}
