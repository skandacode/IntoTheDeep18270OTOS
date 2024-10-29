package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp
public class TeleopSomewhatAutoBlue extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    MecanumDrivetrain drive;

    private enum SampleStates {
        IDLE, EXTEND, RETRACT, WAIT, CLOSE, LIFT, WRIST, OPEN, LOWERLIFT, EJECT
    }
    private enum SpecimenScoreStates{IDLE, INTAKEPOS, INTAKE, CLOSE_CLAW, LIFT, WRISTMOVE, LOWERLIFT, OPENCLAW, RETRACT}
    Intake.SampleColor targetColor = Intake.SampleColor.YELLOW;

    public static int targetLiftPosSample =2900;



    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub:allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        drive = new MecanumDrivetrain(hardwareMap, telemetry, FtcDashboard.getInstance());
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(()->{
                    intake.retract();
                    intake.setPower(0);
                })
                .transition(()->gamepad1.y)

                .state(SampleStates.EXTEND)
                .onEnter(()->intake.intakePosition())
                .transition(()->(intake.getColor()==targetColor), SampleStates.RETRACT)
                .transition(()->{
                    Intake.SampleColor detected=intake.getColor();
                    if (detected== Intake.SampleColor.NONE){
                        return false;
                    }else{
                        System.out.println(detected.toString());
                        return detected!=targetColor;
                    }
                }
                , SampleStates.EJECT)

                .state(SampleStates.EJECT, true)
                .onEnter(()->intake.eject())
                .transitionTimed(0.7, SampleStates.EXTEND)

                .state(SampleStates.RETRACT)
                .onEnter(()->{
                    intake.retract();
                    intake.setPower(0.15);
                    outtake.transferPos();
                })
                .transition(()->intake.isDone())

                .state(SampleStates.WAIT)
                .onEnter(()->intake.setPower(0.15))
                .transitionTimed(0.5)

                .state(SampleStates.CLOSE)
                .onEnter(()->outtake.closeClaw())
                .transitionTimed(0.2)

                .state(SampleStates.LIFT)
                .onEnter(()->{
                    outtake.setTargetPos(targetLiftPosSample);
                    intake.setPower(-0.2);
                })
                .transitionTimed(0.2)

                .state(SampleStates.WRIST)
                .onEnter(()->{
                    outtake.scorePos();
                    intake.setPower(0);
                })
                .transition(()->gamepad1.right_bumper)

                .state(SampleStates.OPEN)
                .onEnter(()->outtake.openClaw())
                .transitionTimed(1.5)

                .state(SampleStates.LOWERLIFT)
                .onEnter(()->{
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .transition(()->(outtake.atTarget() || gamepad1.y), SampleStates.IDLE)
                .build();
        StateMachine specimenScorer = new StateMachineBuilder()
                    .state(SpecimenScoreStates.IDLE)
                    .transition(()->gamepad1.dpad_up)
                    .state(SpecimenScoreStates.INTAKEPOS)
                    .onEnter(()->{
                        outtake.scorePos();
                        outtake.openClawWide();
                        outtake.setTargetPos(200);
                    })
                    .transitionTimed(0.7)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(()->outtake.setTargetPos(0))
                .transition(()->gamepad1.dpad_down)
                    .state(SpecimenScoreStates.CLOSE_CLAW)
                    .onEnter(()->outtake.closeClaw())
                    .transitionTimed(0.3)
                    .state(SpecimenScoreStates.LIFT)
                    .onEnter(()->outtake.setTargetPos(1350))
                    .transition(()-> (outtake.atTarget() && gamepad1.right_bumper))
                    .state(SpecimenScoreStates.WRISTMOVE)
                    .onEnter(()->outtake.specimenDepo())
                    .transitionTimed(0.1)
                    .state(SpecimenScoreStates.LOWERLIFT)
                    .onEnter(()->outtake.setTargetPos(850))
                    .transitionTimed(0.3)
                    .state(SpecimenScoreStates.OPENCLAW)
                    .onEnter(()-> outtake.openClaw())
                    .transitionTimed(0.2)
                    .state(SpecimenScoreStates.RETRACT)
                    .onEnter(()->{
                        outtake.setTargetPos(0);

                        outtake.transferPos();
                    })
                    .transition(()-> outtake.atTarget(), SpecimenScoreStates.IDLE)
                    .build();
        waitForStart();
        outtake.transferPos();
        outtake.resetEncoder();
        sampleMachine.start();
        specimenScorer.start();
        boolean prevSampleColorToggle=false;
        long prevLoop = System.nanoTime();
        while (opModeIsActive()){
            for (LynxModule hub:allHubs){
                hub.clearBulkCache();
            }
            boolean currSampleColorToggle=gamepad1.dpad_down;
            if (currSampleColorToggle && !prevSampleColorToggle){
                if (targetColor == Intake.SampleColor.YELLOW){
                    targetColor = Intake.SampleColor.BLUE;
                }else if (targetColor == Intake.SampleColor.BLUE){
                    targetColor = Intake.SampleColor.YELLOW;
                }
            }
            if (gamepad1.x){
                if (sampleMachine.getState() == SampleStates.EXTEND){
                    sampleMachine.setState(SampleStates.IDLE);
                    intake.retract();
                    intake.setPower(0);
                }
            }

            if (!gamepad1.left_bumper){
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }else{
                drive.setWeightedPowers(-gamepad1.left_stick_y/3, -gamepad1.left_stick_x/3, -gamepad1.right_stick_x/3);
            }
            sampleMachine.update();
            specimenScorer.update();
            intake.update();
            outtake.update();
            telemetry.addData("target color", targetColor);
            telemetry.addData("State sample", sampleMachine.getState());
            telemetry.addData("Specimen sample", specimenScorer.getState());

            telemetry.addData("Intake color", Arrays.toString(intake.getRawSensorValues()));
            telemetry.addData("Intake distance", intake.getDistance());

            telemetry.addData("Outtake Pos", outtake.getLiftPos());

            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop-prevLoop)/1000000);
            prevLoop=currLoop;

            telemetry.update();
            prevSampleColorToggle=currSampleColorToggle;
        }
    }
}
