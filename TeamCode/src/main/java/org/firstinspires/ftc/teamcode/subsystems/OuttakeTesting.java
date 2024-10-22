package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;
    public static int targetPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                outtake.openClaw();
            }
            if (gamepad1.b){
                outtake.closeClaw();
            }
            if (gamepad1.x){
                outtake.transferPos();
            }
            if (gamepad1.y){
                outtake.scorePos();
            }
            outtake.setTargetPos(targetPos);
            telemetry.addData("outtake pos", outtake.getLiftPos());
            telemetry.update();
            outtake.update();
        }
    }
}
