package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@Autonomous
public class RedAutoSamples extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;
    Outtake outtake;
    public static boolean yPressed=false;
    public static boolean lbPressed=false;
    enum autoStates {PREBUCKET1, BUCKET1, SCORE1,
        PRESAMPLE1, EXTEND1, INTAKE1, PREBUCKET2, SCORE2,
        PRESAMPLE2, EXTEND2, INTAKE2, PREBUCKET3, SCORE3,
        PRESAMPLE3, EXTEND3, INTAKE3, PREBUCKET4, SCORE4,
        PARK
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);
        outtake= new Outtake(hardwareMap, telemetry);

        WayPoint bucketPos1=new WayPoint(new Pose2D(DistanceUnit.INCH, -49, -53, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -52, -55, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos3=new WayPoint(new Pose2D(DistanceUnit.INCH, -52, -59, AngleUnit.DEGREES, 43),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint prepark=new WayPoint(new Pose2D(DistanceUnit.INCH, -34, -8, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));

        WayPoint presample1=new WayPoint(new Pose2D(DistanceUnit.INCH, -47, -56, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample1=new WayPoint(new Pose2D(DistanceUnit.INCH, -47, -51, AngleUnit.DEGREES, 89),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample2=new WayPoint(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample2=new WayPoint(new Pose2D(DistanceUnit.INCH, -57, -51, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample3=new WayPoint(new Pose2D(DistanceUnit.INCH, -51, -52.3, AngleUnit.DEGREES, 124),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample3=new WayPoint(new Pose2D(DistanceUnit.INCH, -54, -45, AngleUnit.DEGREES, 120),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));


        StateMachine sampleMachine = new StateMachineBuilder()
                .state(TeleopSomewhatAutoBlue.SampleStates.IDLE)
                .onEnter(() -> {
                    intake.retract();
                    intake.setPower(0);
                })
                .transition(() -> yPressed)
                .state(TeleopSomewhatAutoBlue.SampleStates.EXTEND)
                .onEnter(()->{
                    intake.setExtended(true);
                    yPressed=false;
                })
                .transitionTimed(0.1)
                .state(TeleopSomewhatAutoBlue.SampleStates.DROP)
                .onEnter(() -> {
                    intake.intakePosition();
                    intake.setPower(0.8);
                })
                .transition(()->intake.getDistance()<4)
                .state(TeleopSomewhatAutoBlue.SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.7);
                    outtake.transferPos();
                })
                .transitionTimed(0.2)
                .state(TeleopSomewhatAutoBlue.SampleStates.OPENCOVER)
                .onEnter(() -> intake.setCover(false))
                .transition(() -> intake.isDone())

                .state(TeleopSomewhatAutoBlue.SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.7)

                .state(TeleopSomewhatAutoBlue.SampleStates.CLOSE)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.2)

                .state(TeleopSomewhatAutoBlue.SampleStates.LIFT)
                .onEnter(() -> outtake.setTargetPos(3000))
                .transitionTimed(0.2)

                .state(TeleopSomewhatAutoBlue.SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePos();
                    intake.setPower(0);
                    intake.setPower(-0.2);
                })
                .transition(() -> lbPressed)

                .state(TeleopSomewhatAutoBlue.SampleStates.OPEN).onEnter(() -> {
                    outtake.openClaw();
                    lbPressed=false;
                }).transitionTimed(1.5)

                .state(TeleopSomewhatAutoBlue.SampleStates.LOWERLIFT).onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                }).transition(() -> (outtake.atTarget() || yPressed), TeleopSomewhatAutoBlue.SampleStates.IDLE).build();
        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.PREBUCKET1)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()-> drive.atTarget() && outtake.getLiftPos()>2900)
                .state(autoStates.BUCKET1)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()-> drive.atTarget())
                .state(autoStates.SCORE1)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.PRESAMPLE1)
                .onEnter(()->drive.setTarget(presample1))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND1)
                .onEnter(()->yPressed=true)
                .transitionTimed(1)
                .state(autoStates.INTAKE1)
                .onEnter(()->drive.setTarget(sample1))
                .transition(()->sampleMachine.getState()== TeleopSomewhatAutoBlue.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET2)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2900)
                .state(autoStates.SCORE2)
                .onEnter(()->{
                    drive.setTarget(bucketPos2);
                    lbPressed=true;
                })
                .transitionTimed(0.7)

                .state(autoStates.PRESAMPLE2)
                .onEnter(()->drive.setTarget(presample2))
                .transition(()-> drive.atTarget())

                .state(autoStates.EXTEND2)
                .onEnter(()->yPressed=true)
                .transitionTimed(1)
                .state(autoStates.INTAKE2)
                .onEnter(()->drive.setTarget(sample2))
                .transition(()->sampleMachine.getState()== TeleopSomewhatAutoBlue.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET3)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2900)
                .state(autoStates.SCORE3)
                .onEnter(()->{
                    drive.setTarget(bucketPos2);
                    lbPressed=true;
                })
                .transitionTimed(0.7)

                .state(autoStates.PRESAMPLE3)
                .onEnter(()->drive.setTarget(presample3))
                .transition(()-> drive.atTarget())

                .state(autoStates.EXTEND3)
                .onEnter(()->yPressed=true)
                .transitionTimed(1)
                .state(autoStates.INTAKE3)
                .onEnter(()->drive.setTarget(sample3))
                .transition(()->sampleMachine.getState()== TeleopSomewhatAutoBlue.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET4)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2900)
                .state(autoStates.SCORE4)
                .onEnter(()->{
                    drive.setTarget(bucketPos2);
                    lbPressed=true;
                })
                .transitionTimed(0.7)
                .state(autoStates.PARK)
                .onEnter(()-> drive.setTarget(prepark))
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        waitForStart();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        drive.calibrateIMU();
        sampleMachine.start();
        autoMachine.start();
        sampleMachine.setState(TeleopSomewhatAutoBlue.SampleStates.RETRACT);
        while (opModeIsActive()){
            autoMachine.update();
            sampleMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            telemetry.update();
        }
    }
}
