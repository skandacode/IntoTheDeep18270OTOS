package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@TeleOp
@Config
public class Teleop extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    public static double liftFF=0.08;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        waitForStart();

        outtake.transferPos();


        while (opModeIsActive()){
            if (gamepad1.right_bumper){
                outtake.openClaw();
            }
            if (gamepad1.left_bumper){
                outtake.closeClaw();
            }
            if (gamepad1.y){
                intake.setExtended(true);
                intake.setFlip(0.61);
                intake.setPower(-0.7);
            }
            if (gamepad1.x){
                intake.setExtended(false);
                intake.setPower(0);
                intake.setFlip(0.5);
            }
            if (gamepad1.b){
                outtake.transferPos();
            }
            if (gamepad1.a){
                outtake.scorePos();
            }
            outtake.setPower(gamepad1.right_trigger-gamepad1.left_trigger+liftFF);

            intake.update();
            telemetry.addData("Intake extend Pos", intake.getExtendoMotorPos());
            telemetry.update();
        }
    }
}
