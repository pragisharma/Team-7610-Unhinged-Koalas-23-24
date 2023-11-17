package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto Practice")
public class Auto_Practice extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm;

    double power = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    double STORAGE_ALPHA_ANGLE;
    final int STORAGE_BETA_TICKS = 0;

    final double PICKUP_ALPHA_ANGLE = 48;
    final int PICKUP_BETA_TICKS = -860;

    @Override
    public void runOpMode() throws InterruptedException {
        tlm = hardwareMap.get(DcMotor.class, "topLeft");
        trm = hardwareMap.get(DcMotor.class, "topRight");
        blm = hardwareMap.get(DcMotor.class, "backLeft");
        brm = hardwareMap.get(DcMotor.class, "backRight");

        tlm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);

        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tlm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        pot = hardwareMap.get(AnalogInput.class, "joint 1 pot");

        joint2 = hardwareMap.get(DcMotor.class, "joint 2");
        claw = hardwareMap.get(Servo.class, "claw");

        //reset encoders to 0
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        STORAGE_ALPHA_ANGLE = (pot.getVoltage()/3.3 * 270);

        //close claw, MOVES ON INIT
        claw.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            // test to see if it will go 12 inches
            moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 12));

//            if(elementSpot == 1) { // right spike
//                moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                // turn right
//                // place pixel
//                moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 31));
//            }else if(elementSpot == 2){ // middle spike (in front)
//                moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                // place pixel
//                moveBackward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 31));
//            }else if(elementSpot == 3){ // left spike
//                moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                // turn left
//                // place pixel
//                moveLeft(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 19));
//                moveBackward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 31));
//            }else { // if cv fails
//                moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 31));
//            }


            sleep(20);

            break;
        }
    }

    public void resetMotors() {
        trm.setPower(0);
        tlm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

    }

    public void moveForward(double power, int dist) {
        while (trm.getCurrentPosition() <= dist) {
            telemetry.addData("TRM CURR POS (forward) ", trm.getCurrentPosition());

            telemetry.update();

            // both right motors were given power * 0.8 and it rotated cw at about 6 in
            trm.setPower(power);    // abs -> (+) => + power to trm
            brm.setPower(power);     // abs -> (+) => + power to brm
            tlm.setPower(power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(power);
            sleep(20);
        }

        telemetry.addLine("to be reset (forward)");
        telemetry.update();


        resetMotors();
        sleep(20);
    }

    public void moveBackward(double power, int dist) {
        while (trm.getCurrentPosition() >= dist) {
            telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
            telemetry.update();
            trm.setPower(-power);    // abs -> (+) => + power to trm
            brm.setPower(-power);     // abs -> (+) => + power to brm
            tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(-power);
            sleep(20);
        }

        telemetry.addLine("to be reset. (backward)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void moveLeft(double power, int dist) {
        while (trm.getCurrentPosition() <= dist) { //ref the - motor
            telemetry.addData("BRM CURR POS (left) ", brm.getCurrentPosition());
            trm.setPower(power);
            brm.setPower(-power);
            blm.setPower(power);
            tlm.setPower(-power);
            sleep(20);

            telemetry.addLine();
            telemetry.update();
        }

        telemetry.addLine("to be reset. (left)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void moveRight(double power, int dist) {
        while (trm.getCurrentPosition() >= dist) { //ref the + motor
            telemetry.addData("BRM CURR POS (right) ", brm.getCurrentPosition());
            trm.setPower(-power);
            brm.setPower(power);
            blm.setPower(-power);
            tlm.setPower(power);
            sleep(20);
        }

        telemetry.addLine("to be reset. (right)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void depositFront() {
        while (opModeIsActive()) {
            double joint1Angle = (pot.getVoltage() / 3.3 * 270);
            int joint2Ticks = joint2.getCurrentPosition();

            if (Math.abs(joint1Angle - PICKUP_ALPHA_ANGLE) >= 5) {
                joint1.setPower(-1 * (PICKUP_ALPHA_ANGLE - joint1Angle));
                joint2.setPower(0);
            } else if (Math.abs(joint2Ticks - PICKUP_BETA_TICKS) >= 30) {
                joint1.setPower(0);
                joint2.setPower(1 * (PICKUP_BETA_TICKS - joint2Ticks));
            } else {
                claw.setPosition(1);
                joint1.setPower(0);
                joint2.setPower(0);
                sleep(500);
                break;
            }

            telemetry.addData("pot voltage", pot.getVoltage());
            telemetry.addData("pot angle", pot.getVoltage()/3.3 * 270);
            telemetry.addData("joint 2 ticks", joint2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("joint 1 power", joint1.getPower());
            telemetry.addData("joint 2 power", joint2.getPower());
            telemetry.addLine();
            telemetry.addData("servo position", claw.getPosition());
            telemetry.update();

            sleep(20);
        }
    }

    //preserves state of claw (so it stays open uh oh)
    public void retractArm() {
        while (opModeIsActive()) {
            double joint1Angle = (pot.getVoltage() / 3.3 * 270);
            int joint2Ticks = joint2.getCurrentPosition();

            if (Math.abs(joint1Angle - STORAGE_ALPHA_ANGLE) >= 5) {
                joint1.setPower(-1 * (STORAGE_ALPHA_ANGLE - joint1Angle));
                joint2.setPower(0);
            } else if (Math.abs(joint2Ticks - STORAGE_BETA_TICKS) >= 30) {
                joint1.setPower(0);
                joint2.setPower(1 * (STORAGE_BETA_TICKS - joint2Ticks));
            } else {
                joint1.setPower(0);
                joint2.setPower(0);
                break;
            }

            telemetry.addData("pot voltage", pot.getVoltage());
            telemetry.addData("pot angle", pot.getVoltage()/3.3 * 270);
            telemetry.addData("joint 2 ticks", joint2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("joint 1 power", joint1.getPower());
            telemetry.addData("joint 2 power", joint2.getPower());
            telemetry.addLine();
            telemetry.addData("servo position", claw.getPosition());
            telemetry.update();

            sleep(20);
        }
    }
}