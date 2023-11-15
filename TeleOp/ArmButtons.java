package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ArmButtons.ArmStates.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Bell Qualifier Mech Code")
public class ArmButtons extends LinearOpMode {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, L1_DEPLOY, L2_DEPLOY, PICKUP, L2_RETRACT, L1_RETRACT, SECOND_LINE}
    ArmStates armState = STORAGE;

    double targetAlpha;
    int targetBeta;

    //TODO get positions
    double STORAGE_ALPHA_ANGLE;
    final int STORAGE_BETA_TICKS = 0;

    final double PICKUP_ALPHA_ANGLE = 48;
    final int PICKUP_BETA_TICKS = -860;

    final double FIRST_ALPHA_ANGLE = 0;
    final int FIRST_BETA_TICKS = 0;

    final double SECOND_ALPHA_ANGLE = 0;
    final int SECOND_BETA_TICKS = 0;

    final double THIRD_ALPHA_ANGLE = 0;
    final int THIRD_BETA_TICKS = 0;

    @Override
    public void runOpMode() {
        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        pot = hardwareMap.get(AnalogInput.class, "joint 1 pot");

        STORAGE_ALPHA_ANGLE = (pot.getVoltage()/3.3 * 270);

        joint2 = hardwareMap.get(DcMotor.class, "joint 2");
        claw = hardwareMap.get(Servo.class, "claw");

        //reset encoders to 0
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //no moving
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            arm();
            claw();

            //emergency exit
            if (gamepad2.back) {
                return;
            }

            telemetry.addData("pot voltage", pot.getVoltage());
            telemetry.addData("pot angle (calculated)", pot.getVoltage()/3.3 * 270);
            telemetry.addData("joint 2 ticks", joint2.getCurrentPosition());
            telemetry.addData("arm state", armState);
            telemetry.addLine();
            telemetry.addData("joint 1 power", joint1.getPower());
            telemetry.addData("joint 2 power", joint2.getPower());
            telemetry.addLine();
            telemetry.addData("claw position (1 = open)", claw.getPosition());
            telemetry.update();

            sleep(20);
        }
    }

    public void arm() {
        //handy variables
        double joint1Angle = (pot.getVoltage() / 3.3 * 270);
        int joint2Ticks = joint2.getCurrentPosition();

        //STATE MACHINE TIME WHOOOOO

        //TODO make transitions to line 2 (press x)
        if (armState == STORAGE) {
            joint1.setPower(0);
            joint2.setPower(0);

            if (!bClose(joint2Ticks, targetBeta)) {
                armState = L2_RETRACT;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (!aClose(joint1Angle, targetAlpha)) {
                armState = L1_RETRACT;
                targetAlpha = STORAGE_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (gamepad2.a) {
                armState = L1_DEPLOY;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (gamepad2.x) {
                armState = L1_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L1 close, L2 close, no buttons
                armState = STORAGE;
            }
        } else if (armState == L1_DEPLOY) {
            joint1.setPower(1 * (targetAlpha - joint1Angle));
            joint2.setPower(0);

            if (!gamepad2.a && !gamepad2.x) {
                armState = L1_RETRACT;
                targetAlpha = STORAGE_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (aClose(joint1Angle, targetAlpha)) {
                if (gamepad2.a) {
                    armState = L2_DEPLOY;
                    targetAlpha = PICKUP_ALPHA_ANGLE;
                    targetBeta = PICKUP_BETA_TICKS;
                } else if (gamepad2.x) {
                    armState = L2_DEPLOY;
                    targetAlpha = SECOND_ALPHA_ANGLE;
                    targetBeta = SECOND_BETA_TICKS;
                }
            } else if (gamepad2.x) {
                armState = L1_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L2 close, L1 not close, L2 not close, a
                armState = L1_DEPLOY;
            }
        } else if (armState == L2_DEPLOY) {
            joint1.setPower(0);
            joint2.setPower(1 * (targetBeta - joint2Ticks)); //might need negative coefficient

            if (!gamepad2.a && !gamepad2.x) {
                armState = L2_RETRACT;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (!aClose(joint1Angle, targetAlpha)) {
                if (gamepad2.a) {
                    armState = L1_DEPLOY;
                    targetAlpha = PICKUP_ALPHA_ANGLE;
                    targetBeta = STORAGE_BETA_TICKS;
                } else if (gamepad2.x) {
                    armState = L1_DEPLOY;
                    targetAlpha = SECOND_ALPHA_ANGLE;
                    targetBeta = STORAGE_BETA_TICKS;
                }
            } else if (bClose(joint2Ticks, targetBeta)) {
                if (gamepad2.a) {
                    armState = PICKUP;
                    targetAlpha = PICKUP_ALPHA_ANGLE;
                    targetBeta = PICKUP_BETA_TICKS;
                } else if (gamepad2.x) {
                    armState = SECOND_LINE;
                    targetAlpha = SECOND_ALPHA_ANGLE;
                    targetBeta = SECOND_BETA_TICKS;
                }
            } else { //L2 not close, L1 close, a
                armState = L2_DEPLOY;
            }
        } else if (armState == PICKUP) {
            joint1.setPower(0);
            joint2.setPower(0);

            if (!aClose(joint1Angle, targetAlpha)) {
                armState = L1_DEPLOY;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (!bClose(joint2Ticks, targetBeta)) {
                armState = L2_DEPLOY;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = PICKUP_BETA_TICKS;
            } else if (!gamepad2.a) {
                armState = L2_RETRACT;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L1 close, L2 close, a
                armState = PICKUP;
            }
        } else if (armState == SECOND_LINE) {
            joint1.setPower(0);
            joint2.setPower(0);

            if (!aClose(joint1Angle, targetAlpha)) {
                armState = L1_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (!bClose(joint2Ticks, targetBeta)) {
                armState = L2_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = SECOND_BETA_TICKS;
                //TODO everything down here needs to be fixed D:
            } else if (!gamepad2.x) { //if a: gets sent to L2_RETRACT, then redirected to PICKUP (hopefully)
                armState = L2_RETRACT;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L1 close, L2 close, a
                armState = SECOND_LINE;
            }
        } else if (armState == L2_RETRACT) {
            joint1.setPower(0);
            joint2.setPower(1 * (targetBeta - joint2Ticks));

            if (gamepad2.a) {
                armState = L2_DEPLOY;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = PICKUP_BETA_TICKS;
            } else if (gamepad2.x) {
                armState = L2_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = SECOND_BETA_TICKS;
            } else if (bClose(joint2Ticks, targetBeta)) { //no press
                armState = L1_RETRACT;
                targetAlpha = STORAGE_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L1 close, L1 not close, L2 not close
                armState = L2_RETRACT;
            }
        } else if (armState == L1_RETRACT) {
            joint1.setPower(1 * (targetAlpha - joint1Angle));
            joint2.setPower(0);

            if (gamepad2.a) {
                armState = L1_DEPLOY;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (gamepad2.x) {
                armState = L2_DEPLOY;
                targetAlpha = SECOND_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (!bClose(joint2Ticks, targetBeta)) {
                armState = L2_RETRACT;
                targetAlpha = PICKUP_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else if (aClose(joint1Angle, targetAlpha)) {
                armState = STORAGE;
                targetAlpha = STORAGE_ALPHA_ANGLE;
                targetBeta = STORAGE_BETA_TICKS;
            } else { //L1 not close, L2 close, no buttons
                armState = L1_RETRACT;
            }
        }
    }

    public void claw() {
        //open when pressed
        if (gamepad2.right_bumper) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
    }

    //handy method
    public boolean aClose(double a, double pa) {
        //angles
        return Math.abs(a - pa) <= 10;
    }

    public boolean bClose(double b, double pb) {
        //ticks
        return Math.abs(b - pb) <= 100;
    }
}
