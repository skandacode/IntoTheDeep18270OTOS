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
public class testing extends LinearOpMode {
    CachedMotorEx intakeMotor, extendoMotor, lift1, lift2;
    CachedMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    Servo intakeFlip1, intakeFlip2, depositFlip1, depositFlip2, wrist, claw, intakeCover;

    public static double intakePower=0;
    public static double extendoPower=0;

    public static double outtakePower=0;

    public static double intakeFlip1Pos=0.5;
    public static double intakeFlip2Pos=0.5;
    public static double intakeCoverPos = 0.5;

    public static double depositFlip1Pos=0.4;
    public static double depositFlip2Pos=0.6;

    public static double wristPos=0.5;
    public static double clawPos=0.78;

    public static double frontLeftPower = 0;
    public static double backLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backRightPower = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeMotor= new CachedMotorEx(hardwareMap, "intakeMotor");
        extendoMotor= new CachedMotorEx(hardwareMap, "extendoMotor");
        intakeFlip1=hardwareMap.servo.get("intakeFlip1");
        intakeFlip2=hardwareMap.servo.get("intakeFlip2");
        intakeCover = hardwareMap.servo.get("intakecover");

        depositFlip1=hardwareMap.servo.get("depositFlip1");
        depositFlip2=hardwareMap.servo.get("depositFlip2");
        lift1= new CachedMotorEx(hardwareMap, "leftSlideStack");
        lift2= new CachedMotorEx(hardwareMap, "rightSlideStack");

        wrist=hardwareMap.servo.get("wrist");
        claw=hardwareMap.servo.get("claw");

        frontLeftMotor=new CachedMotorEx(hardwareMap, "frontleft");
        frontRightMotor=new CachedMotorEx(hardwareMap, "frontright");
        backLeftMotor=new CachedMotorEx(hardwareMap, "backleft");
        backRightMotor=new CachedMotorEx(hardwareMap, "backright");


        waitForStart();
        while (opModeIsActive()){
            intakeFlip1.setPosition(intakeFlip1Pos);
            intakeFlip2.setPosition(intakeFlip2Pos);
            intakeCover.setPosition(intakeCoverPos);

            intakeMotor.setPower(intakePower);
            extendoMotor.setPower(extendoPower);

            depositFlip1.setPosition(depositFlip1Pos);
            depositFlip2.setPosition(depositFlip2Pos);

            lift1.setPower(outtakePower);
            lift2.setPower(-outtakePower);

            wrist.setPosition(wristPos);
            claw.setPosition(clawPos);

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);


            telemetry.addData("intakePos", extendoMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
