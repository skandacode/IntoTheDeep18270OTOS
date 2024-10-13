package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);
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
            outtake.setPower(-gamepad1.left_stick_y+0.05);
        }
    }
}
