package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto Practice")
public class Auto_Practice extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm;

    double power = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

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
            sleep(200);
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
            sleep(200);
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
            sleep(200);

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
            sleep(200);
        }

        telemetry.addLine("to be reset. (right)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

}