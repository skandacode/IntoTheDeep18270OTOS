package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Intake implements Subsystem {
    CachedMotorEx intakeMotor, extendoMotor;
    ServoImplEx intakeServoLeft, intakeServoRight;

    private boolean isExtended=false;

    public Intake(HardwareMap hwMap){
        intakeMotor = new CachedMotorEx(hwMap, "intakeMotor");
        extendoMotor = new CachedMotorEx(hwMap, "extendoMotor");

        intakeServoLeft= (ServoImplEx) hwMap.servo.get("intakeFlip2");
        intakeServoRight= (ServoImplEx) hwMap.servo.get("intakeFlip1");

        extendoMotor.resetEncoder();

    }

    @Override
    public void update() {
        //bang bang controller
        if (isExtended){
            if (getExtendoMotorPos()>-500){
                setExtendo(-1);
            }else{
                setExtendo(0);
            }
        } else{
            if (getExtendoMotorPos()<-30){
                setExtendo(1);
            }else{
                setExtendo(0);
            }
        }
    }
    public void setPower(double power){
        intakeMotor.setPower(power);
    }
    void setExtendo(double power){
        extendoMotor.setPower(power);
    }
    public void setFlip(double pos){
        intakeServoLeft.setPosition(pos);
        intakeServoRight.setPosition(1-pos);
    }
    public void enableServos(){
        intakeServoLeft.setPwmEnable();
        intakeServoRight.setPwmEnable();
    }
    public void disableServos(){
        intakeServoLeft.setPwmDisable();
        intakeServoRight.setPwmDisable();
    }
    public int getExtendoMotorPos(){
        return extendoMotor.getCurrentPosition();
    }

    public void setExtended(boolean extended) {
        isExtended = extended;
    }
    public void intakePosition(){
        setExtended(true);
        setFlip(0.61);
        setPower(-0.7);
    }
    public void retract(){
        setExtended(false);
        setPower(0);
        setFlip(0.5);
    }
}
