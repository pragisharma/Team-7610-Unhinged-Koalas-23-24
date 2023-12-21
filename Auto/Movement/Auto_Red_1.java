package org.firstinspires.ftc.teamcode.Auto.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous(name = "Red Alliance Pos 1") // closer to the backdrop
public class Auto_Red_1 extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm;

    double power = 0.4;
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 0.855;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    IMU imu;
    IMU.Parameters myIMUparameters;



    Orientation angles;

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



        imu = hardwareMap.get(IMU.class, "imu");

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                -45,
                                0  // acquisitionTime, not used
                        )
                )
        );



        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            // REMINDERS: PASS IN NEGATIVE DISTANCE FOR MOVE RIGHT AND MOVE BACKWARD
            // issue with right: moves 27 inches NOT 12 (idk why), try conversion factor of 1/2.25 for dist
            //moveRight(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 12)));

            moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 24));


            //case 1 (middle spike - 2)

            // if custom game element is present
                // place pixel
                // the right is sooo screwed up >> (might be easier to rotate and go backward/forward)
                // moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 40));

            //case 2 (right spike - 1)

            // orginally: turn(power, -88.00); // 2 less bc of rounding errors

            // turn(power, -30.00); // 30 goes too much 20 goes too less for a 90 degree turn

            // else if custom game element is present
                // place pixel
                // moveBackward(power, (int)(-(trm.getCurrentPosition() + COUNTS_PER_INCH * 30)));

            //case 3 (left spike - 3)

            // turn(power, 180.00);
            // else if custom game element is present
                // place pixel
                // moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 24));
                // moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 40));

            // error case (if nothing is scanned)
            // else
                // moveRight(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 24));
                // moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH * 40));
                // drop pixel in back area


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


    public void turn(double power, double deg){
        // reset yaw each time?
        // imu.resetYaw();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
//        telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
//        telemetry.update();

        if(deg >= 0){
            // while we want to rotate it cw (+ angle)
            while(-deg <  robotOrientation.getYaw(AngleUnit.DEGREES)) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                sleep(20);

                trm.setPower(-power);
                brm.setPower(-power);
                tlm.setPower(power);
                blm.setPower(power);
                sleep(200);
            }
        }

//        else if(deg < 0){
//            // while we want to rotate it ccw (- angle)
//            while(deg >= robotOrientation.getYaw(AngleUnit.DEGREES)) {
//                robotOrientation = imu.getRobotYawPitchRollAngles();
//                telemetry.addData("CURRENT DEGREES: ", robotOrientation.getYaw(AngleUnit.DEGREES));
//                telemetry.update();
//                trm.setPower(power);
//                brm.setPower(power);
//                tlm.setPower(-power);
//                blm.setPower(-power);
//                sleep(20);
//            }
//        }
//
//
//        resetMotors();
        sleep(20);

    }
    // u hafta upload the code again b/c its a static variable issue for backwards and right
    public void moveBackward(double power, int dist) {
        //telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
        // orginially greater than or equal to
        // dist = 637.2203205144
        // initial trm position is 180
        while (trm.getCurrentPosition() >= dist) {
            //telemetry.addLine("INSIDE LOOP ");
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

    // problem child
    public void moveRight(double power, int dist) {
        // moves 28 inches when its supposed to move 12
        while (trm.getCurrentPosition() >= dist) { //ref the + motor
            telemetry.addData("TRM CURR POS (right) ", trm.getCurrentPosition());
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