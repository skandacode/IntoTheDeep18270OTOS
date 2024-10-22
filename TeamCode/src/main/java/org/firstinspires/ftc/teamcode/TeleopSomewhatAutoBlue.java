package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@TeleOp
public class TeleopSomewhatAutoBlue extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    MecanumDrivetrain drive;

    private enum SampleStates {
        IDLE, EXTEND, RETRACT, WAIT, CLOSE, LIFT, WRIST, OPEN, LOWERLIFT, EJECT
    }
    Intake.SampleColor targetColor = Intake.SampleColor.YELLOW;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        drive = new MecanumDrivetrain(hardwareMap, telemetry, FtcDashboard.getInstance());
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .transition(()->gamepad1.y)

                .state(SampleStates.EXTEND)
                .onEnter(()->intake.intakePosition())
                .transition(()->(intake.getColor()==targetColor), SampleStates.RETRACT)
                .transition(()->{
                    Intake.SampleColor detected=intake.getColor();
                    if (detected== Intake.SampleColor.NONE){
                        return false;
                    }else{
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
                    outtake.setTargetPos(800);
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
                .transitionTimed(0.5)

                .state(SampleStates.LOWERLIFT)
                .onEnter(()->{
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .transition(()->(outtake.atTarget() || gamepad1.y), SampleStates.IDLE)
                .build();
        waitForStart();
        outtake.transferPos();
        sampleMachine.start();
        boolean prevSampleColorToggle=false;
        while (opModeIsActive()){
            boolean currSampleColorToggle=gamepad1.dpad_down;
            if (currSampleColorToggle && !prevSampleColorToggle){
                if (targetColor == Intake.SampleColor.YELLOW){
                    targetColor = Intake.SampleColor.BLUE;
                }else if (targetColor == Intake.SampleColor.BLUE){
                    targetColor = Intake.SampleColor.YELLOW;
                }
            }
            drive.setWeightedPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            sampleMachine.update();
            intake.update();
            outtake.update();
            telemetry.addData("target color", targetColor);
            telemetry.addData("State", sampleMachine.getState());
            telemetry.addData("Intake color", intake.getColor());

            telemetry.update();
            prevSampleColorToggle=currSampleColorToggle;
        }
    }
}
