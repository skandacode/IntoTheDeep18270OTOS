package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Outtake implements Subsystem{
    private CachedMotorEx leftLift, rightLift;
    private Servo depositFlip1, depositFlip2, wrist, claw;

    private int targetPos=0;

    private PIDFController controller;

    public Outtake(HardwareMap hwMap){
        leftLift= new CachedMotorEx(hwMap, "leftSlideStack");
        rightLift= new CachedMotorEx(hwMap, "rightSlideStack");

        depositFlip1=hwMap.servo.get("depositFlip1");
        depositFlip2=hwMap.servo.get("depositFlip2");

        wrist=hwMap.servo.get("wrist");
        claw=hwMap.servo.get("claw");

        leftLift.resetEncoder();

        controller = new PIDFController(0.01, 0, 0, 0.08);

        controller.setTolerance(10);
    }

    public void setPower(double power){
        leftLift.setPower(-power);
        rightLift.setPower(power);
    }

    public int getLiftPos(){
        return leftLift.getCurrentPosition();
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
        setFlipPos(0.96);
        openClaw();
    }
    public void scorePos(){
        setFlipPos(0.4);
        closeClaw();
    }

    @Override
    public void update() {
        double controller_output=controller.calculate(getLiftPos());

        if (getLiftPos()>targetPos){
            controller_output-=0.05;
        }
        if (getLiftPos()<targetPos){
            controller_output+=0.05;
        }

        setPower(controller_output);

    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }
}
