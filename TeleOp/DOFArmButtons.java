package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Bell Qualifier Mech Code")
public class DOFArmButtons extends LinearOpMode {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    //positions
    final double STORAGE_ALPHA_ANGLE = 0;
    final int STORAGE_BETA_TICKS = 0;

    final double PICKUP_ALPHA_ANGLE = 0;
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

        joint2 = hardwareMap.get(DcMotor.class, "joint 2");
        claw = hardwareMap.get(Servo.class, "claw");

        //reset encoders to 0
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //no moving
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            arm();
            claw();

            telemetry.addData("pot voltage", pot.getVoltage());
            telemetry.addData("pot angle", pot.getVoltage()/3.3 * 270);
            telemetry.addData("joint 2 ticks", joint2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("joint 1 power", joint1.getPower());
            telemetry.addData("joint 2 power", joint2.getPower());
            telemetry.addLine();
            telemetry.addData("claw position, 1 = open", claw.getPosition());
            telemetry.update();

            sleep(20);
        }
    }

    public void arm() {
        //handy variables
        double joint1Angle = 250 - (pot.getVoltage()/3.3 * 270); //storage is 250 smh
        int joint2Ticks = joint2.getCurrentPosition();

        if (gamepad2.a) {
            //pickup
            if (closeToPos(joint1Angle, joint2Ticks, PICKUP_ALPHA_ANGLE, PICKUP_BETA_TICKS)) {
                joint1.setPower(PICKUP_ALPHA_ANGLE - joint1Angle);
                joint2.setPower(0.5 * (PICKUP_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0.5 * (PICKUP_BETA_TICKS - joint2Ticks));
            }
        } else if (gamepad2.b) {
            //line 1
            if (closeToPos(joint1Angle, joint2Ticks, FIRST_ALPHA_ANGLE, FIRST_BETA_TICKS)) {
                joint1.setPower(FIRST_ALPHA_ANGLE - joint1Angle);
                joint2.setPower(0.5 * (FIRST_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0.5 * (FIRST_BETA_TICKS - joint2Ticks));
            }
        } else if (gamepad2.x) {
            //line 2
            if (closeToPos(joint1Angle, joint2Ticks, SECOND_ALPHA_ANGLE, SECOND_BETA_TICKS)) {
                joint1.setPower(SECOND_ALPHA_ANGLE - joint1Angle);
                joint2.setPower(0.5 * (SECOND_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0.5 * (SECOND_BETA_TICKS - joint2Ticks));
            }
        } else if (gamepad2.y) {
            //line 3
            if (closeToPos(joint1Angle, joint2Ticks, THIRD_ALPHA_ANGLE, THIRD_BETA_TICKS)) {
                joint1.setPower(THIRD_ALPHA_ANGLE - joint1Angle);
                joint2.setPower(0.5 * (THIRD_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0.5 * (THIRD_BETA_TICKS - joint2Ticks));
            }
        } else {
            //storage
            if (closeToPos(joint1Angle, joint2Ticks, STORAGE_ALPHA_ANGLE, STORAGE_BETA_TICKS)) {
                joint1.setPower(STORAGE_ALPHA_ANGLE - joint1Angle);
                joint2.setPower(0.5 * (STORAGE_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0.5 * (STORAGE_BETA_TICKS - joint2Ticks));
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
    public boolean closeToPos(double a, double b, double pa, double pb) {
        //a = alpha (angle), b = beta (ticks)
        //pa = target a (angle), pb = target b (ticks)
        boolean alphaClose = Math.abs(a - pa) <= 10;
        boolean betaClose = Math.abs(b - pb) <= 100;
        return alphaClose || betaClose;
    }
}
