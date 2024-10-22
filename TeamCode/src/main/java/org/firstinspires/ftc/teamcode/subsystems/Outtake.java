package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Outtake implements Subsystem{
    private CachedMotorEx leftLift, rightLift;
    private Servo depositFlip1, depositFlip2, wrist, claw;

    private int targetPos=0;

    public enum possibleHeights {HIGHBASKET, LOWBASKET, HIGHSPECIMEN, LOWSPECIMEN, HUMANPLAYER}

    private PIDFController controller;
    Telemetry telemetry;

    public Outtake(HardwareMap hwMap, Telemetry telemetry){
        leftLift= new CachedMotorEx(hwMap, "leftSlideStack");
        rightLift= new CachedMotorEx(hwMap, "rightSlideStack");

        depositFlip1=hwMap.servo.get("depositFlip1");
        depositFlip2=hwMap.servo.get("depositFlip2");

        wrist=hwMap.servo.get("wrist");
        claw=hwMap.servo.get("claw");

        leftLift.resetEncoder();

        controller = new PIDFController(0.01, 0, 0, 0);

        controller.setTolerance(10);

        this.telemetry=telemetry;
    }

    public void setPower(double power){
        leftLift.setPower(-power);
        rightLift.setPower(power);
    }

    public int getLiftPos(){
        return -rightLift.getCurrentPosition();
    }

    public void openClaw(){
        claw.setPosition(0.78);
    }

    public void closeClaw(){
        claw.setPosition(0.92);
    }

    public void setFlipPos(double pos){
        depositFlip1.setPosition(pos);
        depositFlip2.setPosition(1-pos);
    }

    public void setWristPos(double pos){
        wrist.setPosition(pos);
    }

    public void transferPos(){
        setFlipPos(0.885);
        setWristPos(0.08);
        openClaw();
    }
    public void scorePos(){
        setFlipPos(0.4);
        setWristPos(0.7);
        closeClaw();
    }

    @Override
    public void update() {
        double controller_output=controller.calculate(getLiftPos());

//        if (getLiftPos()>targetPos){
//            controller_output-=0.05;
//        }
//        else if (getLiftPos()<targetPos){
//            controller_output+=0.05;
//        }
        telemetry.addData("Outtake applied power", controller_output);
        setPower(controller_output);

    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
        controller.setSetPoint(targetPos);
    }
}
