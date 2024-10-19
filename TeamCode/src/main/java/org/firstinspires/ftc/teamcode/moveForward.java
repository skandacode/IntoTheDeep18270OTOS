package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
@Config
public class moveForward extends LinearOpMode {
    //HardwareMap hwMap;
    MotorEx leftFront;
    MotorEx leftBack;
    MotorEx rightFront;
    MotorEx rightBack;

    @Override
    public void runOpMode() {
        leftFront=new MotorEx(hardwareMap, "frontleft");
        leftBack=new MotorEx(hardwareMap, "backleft");
        rightFront=new MotorEx(hardwareMap, "frontright");
        rightBack=new MotorEx(hardwareMap, "backright");


    waitForStart();
    while (opModeIsActive()) {
        leftFront.set(1);
        leftBack.set(-1);
        rightFront.set(1);
        rightBack.set(-1);

    }
}}