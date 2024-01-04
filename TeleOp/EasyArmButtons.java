package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.EasyArmButtons.ArmStates.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Button-controlled arm")
public class EasyArmButtons extends LinearOpMode {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, L1_DEPLOY_HIGH, L2_PICKUP, PICKUP, L1_RETRACT_HIGH, L2_STORAGE, //yeah this part's a mess
        FIRST_LINE, SECOND_LINE, THIRD_LINE,
        LINE_RETRACT}
    ArmStates armState = STORAGE;

    //TODO: WHY IS STORAGE ZERO
    double STORAGE_ALPHA_ANGLE; // set when OpMode is initialized
    final int STORAGE_BETA_TICKS = 0;

    final double PICKUP_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 38;
    final double PICKUP_SECONDARY_ALPHA = STORAGE_ALPHA_ANGLE + 43; // the one you lift to for joint 2 to have clearance
    final int PICKUP_BETA_TICKS = 1700;

    final double FIRST_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 67;
    final int FIRST_BETA_TICKS = 403;

    final double SECOND_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 68;
    final int SECOND_BETA_TICKS = 574;

    final double THIRD_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 95;
    final int THIRD_BETA_TICKS = 1095;

    //arm control constants
    //final double aP = 0.01; //unused
    final double bP = 0.005;
    double maxArmPower = 1.0;

    @Override
    public void runOpMode() {
        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        pot = hardwareMap.get(AnalogInput.class, "joint 1 pot");
        STORAGE_ALPHA_ANGLE = (pot.getVoltage() * 270 / 3.3);

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
            telemetry.addData("pot angle (calculated)", pot.getVoltage() * 270 / 3.3);
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
        double alpha = (pot.getVoltage() * 270 / 3.3);
        double beta = joint2.getCurrentPosition();

        /*
        A little chunk on why transitioning between STORAGE and PICKUP is hell.

        We use 3 movements to get the arm to move from STORAGE to PICKUP:
            1) joint 1 to HIGHER position (than the target)
            2) joint 2 to correct pos
            3) joint 1 to correct pos (lower)

        similarly, you have to reverse the process to get it back into storage:
            1) joint 1 to HIGHER
            2) joint 2 to storage
            3) joint 1 to storage

        i'm pretty sure you can't combine any of the three movements which makes this really annoying
        so in summary, i'm pretty sure that, aside from STORAGE and PICKUP, 4 intermediate stages are needed.
        just to move between them.
        WHYYYYYYY (so the arm doesn't self-destruct trying to rotate into the ground)
         */

        //the hard part
        double joint1Power;
        if (armState == STORAGE) {
            joint1Power = setJoint1Power(alpha, STORAGE_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = L1_DEPLOY_HIGH;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else if (!bClose(beta, STORAGE_BETA_TICKS)) {
                armState = L2_STORAGE;
            } else {
                armState = STORAGE;
            }
        } else if (armState == L1_DEPLOY_HIGH) {
            joint1Power = setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (!gamepad2.a && !gamepad2.x) {
                armState = STORAGE;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else if (aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                armState = L2_PICKUP;
            } else {
                armState = L1_DEPLOY_HIGH;
            }
        } else if (armState == L2_PICKUP) {
            joint1Power = setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (!gamepad2.a && !gamepad2.x) {
                armState = L2_STORAGE;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else if (!aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                armState = L1_DEPLOY_HIGH;
            } else if (bClose(beta, PICKUP_BETA_TICKS)) {
                armState = PICKUP;
            } else {
                armState = L2_PICKUP;
            }
        } else if (armState == PICKUP) {
            joint1Power = setJoint1Power(alpha, PICKUP_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = PICKUP;
            } else if (!bClose(beta, PICKUP_BETA_TICKS)) {
                armState = PICKUP;
            } else {
                armState = L1_RETRACT_HIGH;
            }
        } else if (armState == L1_RETRACT_HIGH) {
            joint1Power = setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = PICKUP;
            } else if (aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                armState = L2_STORAGE;
            } else {
                armState = L1_RETRACT_HIGH;
            }
        } else if (armState == L2_STORAGE) { //"exit state" to set lines
            joint1Power = setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA);

            joint1.setPower(joint1Power);
            //any power set here is overridden when transitioning to set lines anyways
            joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = L2_PICKUP;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else if (!aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                armState = L1_RETRACT_HIGH;
            } else if (bClose(beta, STORAGE_BETA_TICKS)) {
                armState = STORAGE;
            } else {
                armState = L2_STORAGE;
            }
        }

        //fortunately the rest of the states are easy, i'm just too lazy to get the rest of the set lines in
        else if (armState == FIRST_LINE) {
            joint1Power = setJoint1Power(alpha, FIRST_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (FIRST_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = PICKUP;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else {
                armState = LINE_RETRACT;
            }
        } else if (armState == SECOND_LINE) {
            joint1Power = setJoint1Power(alpha, SECOND_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (SECOND_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = PICKUP;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else {
                armState = LINE_RETRACT;
            }
        } else if (armState == THIRD_LINE) {
            joint1Power = setJoint1Power(alpha, THIRD_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (THIRD_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.a) {
                armState = PICKUP;
            } else if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else {
                armState = LINE_RETRACT;
            }
        }

        else if (armState == LINE_RETRACT) {
            joint1Power = setJoint1Power(alpha, STORAGE_ALPHA_ANGLE);

            joint1.setPower(joint1Power);
            joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

            if (gamepad2.b) {
                armState = FIRST_LINE;
            } else if (gamepad2.x) {
                armState = SECOND_LINE;
            } else if (gamepad2.y) {
                armState = THIRD_LINE;
            } else if (aClose(alpha, STORAGE_ALPHA_ANGLE) && bClose(beta, STORAGE_BETA_TICKS)) {
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

    /*-- helpers --*/
    public double setJoint1Power(double alpha, double target) {
        if (aClose(alpha, target)) {
            return 0;
        //a little fudge since it would do the flop
        } else if (target - alpha > 0) {
            return -maxArmPower * Math.abs(target-alpha)/alpha;
        } else {
            return maxArmPower * Math.abs(target-alpha)/alpha;
        }
    }

    public boolean aClose(double actual, double target) {
        return Math.abs(target - actual) <= 1;
    }

    public boolean bClose(double actual, double target) {
        return Math.abs(target - actual) <= 10;
    }
}