package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Intake implements Subsystem {
    CachedMotorEx intakeMotor;
    public Intake(HardwareMap hwMap){
        intakeMotor = new CachedMotorEx(hwMap, "extendoMotor");

    }

    @Override
    public void update() {
        //pid with friction feedforward

    }
}
