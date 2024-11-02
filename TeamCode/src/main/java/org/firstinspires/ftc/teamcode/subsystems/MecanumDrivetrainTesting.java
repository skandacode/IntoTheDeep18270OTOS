package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;

@Config
@TeleOp
public class MecanumDrivetrainTesting extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);

        drive.setPosition(new Pose2D(DistanceUnit.INCH, 36, 0, AngleUnit.DEGREES, 90));
        WayPoint point1=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, 18, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, 0, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));

        waitForStart();
        drive.setTarget(startPoint);
        intake.retract();
        while (opModeIsActive()){
            drive.update();
            drive.updatePIDS();
            if (gamepad1.a){
                drive.setTarget(point1);
            }
            if (gamepad1.b){
                drive.setTarget(startPoint);
            }
            intake.update();
            telemetry.update();
        }
    }
}
