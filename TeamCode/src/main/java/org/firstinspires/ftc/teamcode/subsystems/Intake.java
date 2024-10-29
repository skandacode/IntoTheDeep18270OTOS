package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Intake implements Subsystem {
    CachedMotorEx intakeMotor, extendoMotor;
    ServoImplEx intakeServoLeft, intakeServoRight;
    Servo cover;
    TouchSensor intakeEnd;
    RevColorSensorV3 intakecolor;

    private boolean isExtended=false;
    private boolean isDone = true;

    public enum SampleColor {RED, BLUE, YELLOW, NONE}

    public Intake(HardwareMap hwMap){
        intakeMotor = new CachedMotorEx(hwMap, "intakeMotor");
        extendoMotor = new CachedMotorEx(hwMap, "extendoMotor");

        intakeServoLeft= (ServoImplEx) hwMap.servo.get("intakeFlip2");
        intakeServoRight= (ServoImplEx) hwMap.servo.get("intakeFlip1");
        cover = hwMap.servo.get("intakecover");

        intakeEnd = hwMap.get(TouchSensor.class, "intakeend0");
        intakecolor = hwMap.get(RevColorSensorV3.class, "intakecolor");

        extendoMotor.resetEncoder();

    }

    @Override
    public void update() {
        //bang bang controller
        if (intakeEnd.isPressed()){
            extendoMotor.resetEncoder();
        }
        if (isExtended){
            if (getExtendoMotorPos()>-450){
                setExtendo(-1);
                isDone=false;
            }else{
                setExtendo(0);
                isDone=true;
            }
        } else{
            if (intakeEnd.isPressed()){
                setExtendo(0);
                isDone=true;
            }else{
                setExtendo(1);
                isDone=false;
            }
        }
    }
    public void setPower(double power){
        intakeMotor.setPower(-power);
    }
    void setExtendo(double power){
        extendoMotor.setPower(power);
    }
    public void setFlip(double pos){
        intakeServoLeft.setPosition(pos);
        intakeServoRight.setPosition(1-pos);
    }
    public void setCover(boolean closed){
        if (closed){
            cover.setPosition(0.9);
        }else{
            cover.setPosition(0.25);
        }
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
        setCover(true);
        setFlip(0.61);
        setPower(1);
    }
    public void eject(){
        setCover(false);
        setFlip(0.38);
    }
    public void retract(){
        setExtended(false);
        setPower(0);
        setCover(false);
        setFlip(0.5);
    }

    public boolean isDone() {
        return isDone;
    }
    public int[] getRawSensorValues() {
        return new int[]{intakecolor.red(), intakecolor.green(), intakecolor.blue()}; // Return RGB as an array
    }
    public double getDistance(){
        return intakecolor.getDistance(DistanceUnit.CM);
    }
    public SampleColor getColor(){
        if (intakecolor.getDistance(DistanceUnit.CM)<4){
            /*
            int blueValue = intakecolor.blue();
            if (blueValue>180){
                return SampleColor.BLUE;
            }
            int redValue = intakecolor.red();
            if (redValue<210){
                return SampleColor.RED;
            }
            return SampleColor.YELLOW;
             */
            int[] rgbValues = getRawSensorValues();
            if (rgbValues[0]>rgbValues[1] && rgbValues[0]>rgbValues[2]){
                return SampleColor.RED;
            }
            if (rgbValues[1]>rgbValues[0] && rgbValues[1]>rgbValues[2]){
                return SampleColor.YELLOW;
            }
            if (rgbValues[2]-100>rgbValues[1] && rgbValues[2]-100>rgbValues[0]){
                return SampleColor.BLUE;
            }

        }else{
            return SampleColor.NONE;
        }
        return SampleColor.NONE;
    }
}
