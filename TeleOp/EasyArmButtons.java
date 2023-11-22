package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.EasyArmButtons.ArmStates.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Easy Arm Button")
public class EasyArmButtons extends LinearOpMode {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, PICKUP, FIRST_LINE, SECOND_LINE, THIRD_LINE}
    ArmStates armState = STORAGE;

    double targetAlpha;
    int targetBeta;

    //TODO get positions
    double STORAGE_ALPHA_ANGLE;
    final int STORAGE_BETA_TICKS = 0;

    final double PICKUP_ALPHA_ANGLE = 48;
    final int PICKUP_BETA_TICKS = -430;

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
            telemetry.addData("claw position (0 = open)", claw.getPosition());
            telemetry.update();

            sleep(20);
        }
    }

    public void arm() {
        double alpha = (pot.getVoltage()/3.3 * 270);
        double beta = joint2.getCurrentPosition();
        if (armState == STORAGE) {
            joint1.setPower(1 * (STORAGE_ALPHA_ANGLE - alpha));
            joint2.setPower(1 * (beta - STORAGE_BETA_TICKS));

            if (gamepad2.a) {
                armState = PICKUP;
            } else {
                armState = STORAGE;
            }
        } else if (armState == PICKUP) {
            joint1.setPower(1 * (PICKUP_ALPHA_ANGLE - alpha));
            joint2.setPower(1 * (beta - PICKUP_BETA_TICKS));

            if (gamepad2.a) {
                armState = PICKUP;
            } else {
                armState = STORAGE;
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
}
