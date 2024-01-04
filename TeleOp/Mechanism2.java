package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.Mechanism2.ArmStates.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "2.0 Arm Code")
public class Mechanism2 extends LinearOpMode {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {
        STORAGE, L1_DEPLOY_HIGH, L2_PICKUP, PICKUP, L1_RETRACT_HIGH, L2_STORAGE, //yeah this part's a mess
        FIRST_LINE, SECOND_LINE, THIRD_LINE,
        LINE_RETRACT
    }

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
            telemetry.addLine();
            telemetry.addData("storage pot angle", STORAGE_ALPHA_ANGLE);
            telemetry.update();

            sleep(20);
        }
    }
}