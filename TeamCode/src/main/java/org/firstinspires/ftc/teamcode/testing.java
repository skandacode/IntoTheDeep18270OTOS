package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

@Config
@TeleOp
@Disabled
public class testing extends LinearOpMode {
    CachedMotorEx intakeMotor, extendoMotor, lift1, lift2;
    Servo intakeFlip1, intakeFlip2, depositFlip1, depositFlip2, wrist, claw;

    public static double intakePower=0;
    public static double extendoPower=0;

    public static double outtakePower=0;

    public static double intakeFlip1Pos=0.5;
    public static double intakeFlip2Pos=0.5;
    public static double depositFlip1Pos=0;
    public static double depositFlip2Pos=0;

    public static double wristPos=0;
    public static double clawPos=0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeMotor= new CachedMotorEx(hardwareMap, "intakeMotor");
        extendoMotor= new CachedMotorEx(hardwareMap, "extendoMotor");
        intakeFlip1=hardwareMap.servo.get("intakeFlip1");
        intakeFlip2=hardwareMap.servo.get("intakeFlip2");
        depositFlip1=hardwareMap.servo.get("depositFlip1");
        depositFlip2=hardwareMap.servo.get("depositFlip2");
        lift1= new CachedMotorEx(hardwareMap, "leftSlideStack");
        lift2= new CachedMotorEx(hardwareMap, "rightSlideStack");

        wrist=hardwareMap.servo.get("wrist");
        claw=hardwareMap.servo.get("claw");

        waitForStart();
        while (opModeIsActive()){
            intakeFlip1.setPosition(intakeFlip1Pos);
            intakeFlip2.setPosition(intakeFlip2Pos);
            intakeMotor.setPower(intakePower);
            extendoMotor.setPower(extendoPower);

            depositFlip1.setPosition(depositFlip1Pos);
            depositFlip2.setPosition(depositFlip2Pos);

            lift1.setPower(outtakePower);
            lift2.setPower(-outtakePower);

            wrist.setPosition(wristPos);
            claw.setPosition(clawPos);

            telemetry.addData("intakePos", extendoMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
