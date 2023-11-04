package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This class describes a TeleOp_6038 that contains a 20 millisecond loop controlling
 * the 20:1 Core Hex Motors used for the wheels and linear slide based on input from a gamepad
 */
@TeleOp(group = "LinearOpMode", name = "TeleOp")
//@Disabled
public class TeleOp1 extends LinearOpMode {

    /* motor declaration */
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    /**
     * New instance of a DriveStateMachine that controls the motor power and drive state
     */
    DriveStateMachine driveSM = new DriveStateMachine();

    // assign local instance variables properties of the state machine
    private int driveState = driveSM.NONE;
    // private int armState;
    private double pow;
    //private double gripperState;

    int DRIVE = driveSM.DRIVE;
    int STRAFE = driveSM.STRAFE;
    int TURN_RIGHT = driveSM.TURN_RIGHT;
    int TURN_LEFT = driveSM.TURN_LEFT;
    int NONE = driveSM.NONE;


    /**
     * This method determines the actions of the robot, it must be implemented because
     * the class extends LinearOpMode
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        /* motor initialization */
        frontRight = hardwareMap.get(DcMotor.class, "topRight");
        frontLeft = hardwareMap.get(DcMotor.class, "topLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //slide = hardwareMap.get(DcMotor.class, "slide");
        //gripper = hardwareMap.get(Servo.class, "gripper");
        //gripper.scaleRange(0, gripperSM.OPEN);


        // set brake and direction (power is intuitive after direction reverse)
        // all + power => robot moves forward
        driveSM.setInitialBehavior();
        //gripper.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {

            // update state machines and instance variables
            driveSM.setDriveState();
            driveState = driveSM.getDriveState();
            pow = driveSM.getPower();
            //gripperSM.setState();

            //armState = armSM.getSlideState(); // DO NOT UPDATE ENCODER COUNT HERE
            //gripperState = gripperSM.getState();


            // setting motor power after attributes are updated
            move();
            //armSM.run();
            //gripperSM.run();

            // update telemetry every iteration of the loop
            // 20 millisecond sleep to avoid control and expansion hub overload
            writeTelemetry();
            telemetry.update();
            sleep(20);

        }
        //slide.setTargetPosition(armSM.BASE);
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //gripper.setPosition(0);

    }
    /**
     * This method sends the power to the motors based on updated instance variables and
     * the variable power which is updated in the main method runOpMode()
     */
    public void move() {
        /* all + => forward
         * all - => backward
         * diagonal wheels same direction => strafe
         * right +, left - =>  turn
         */
        if (driveState == DRIVE) {
            frontRight.setPower(-pow);
            frontLeft.setPower(-pow);
            backRight.setPower(pow);
            backLeft.setPower(pow);
        } else if (driveState == STRAFE){
            frontRight.setPower(pow);
            frontLeft.setPower(-pow);
            backRight.setPower(-pow);
            backLeft.setPower(pow);
        } else if (driveState == TURN_RIGHT) {
            frontRight.setPower(pow);
            frontLeft.setPower(-pow);
            backRight.setPower(pow);
            backLeft.setPower(-pow);
        } else if (driveState == TURN_LEFT) {
            frontRight.setPower(-pow);
            frontLeft.setPower(pow);
            backRight.setPower(-pow);
            backLeft.setPower(pow);
        } else if (driveState == NONE) {
            frontRight.setPower(pow);
            frontLeft.setPower(pow);
            backRight.setPower(pow);
            backLeft.setPower(pow);
        }


    }

    /**
     * This method adds telemetry that describes the current encoder values of the slide and
     * gripper, in addition to the state of each state machine
     * It will not be written to the driver hub unless telemetry.update() is called inside
     * of the whileOpModeIsActive() 20 millisecond loop inside of runOpMode in the main class
     */
    public void writeTelemetry() {
        // drive state telemetry
        String driveStateStr;
        if (driveState == driveSM.DRIVE) {
            driveStateStr = "drive";
        } else if (driveState == driveSM.STRAFE) {
            driveStateStr = "strafe";
        } else if (driveState == driveSM.TURN_RIGHT) {
            driveStateStr = "turn right";
        } else if (driveState == driveSM.TURN_LEFT) {
            driveStateStr = "turn left";
        } else {
            driveStateStr = "none";
        }
        telemetry.addData("drive state: ", driveStateStr);
    }


    /**
     * This class describes a DriveStateMachine with accessible attributes of STRAFE, DRIVE,
     * TURN_RIGHT, TURN_LEFT, and NONE
     */
    class DriveStateMachine {
        private int driveState = 0;
        private double pow = 0.0;
        private double MAX_POWER = 0.7;
        private static final int STRAFE = 0;
        private static final int DRIVE = 1;
        private static final int TURN_RIGHT = 2;
        private static final int TURN_LEFT = 3;
        private static final int NONE = 4;
        private final double JOYSTICK_SENSITIVITY = 0.1;

        private double drive;
        private double strafe;
        private double turnRight;
        private double turnLeft;

        /**
         * Sets the initial behavior of the motors
         * Sets direction, zero power behavior and resets encoder values
         * Zero power behavior should always be brake - motors actively repel any extra force
         * applied instead of drifting after button is released
         * ## Only reset the encoder values of the motor when the encoder cable is plugged in!!
         * if the encoder cable is not plugged in and the encoders are reset, motors
         * revert to factory state and will not run
         */
        public void setInitialBehavior() {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            //slide.setDirection(DcMotor.Direction.REVERSE);
            //gripper.setDirection(Servo.Direction.FORWARD);


            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /**
         * This method sets the drive state of the robot based on joystick input
         */
        public void setDriveState() {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.right_stick_x;
            turnRight = gamepad1.right_trigger;
            turnLeft = gamepad1.left_trigger;

            telemetry.addLine("in");
            telemetry.addData("drive", drive);
            telemetry.addData("strafe", strafe);
            telemetry.update();

            if (drive < -JOYSTICK_SENSITIVITY || drive > JOYSTICK_SENSITIVITY) {
                driveState = DRIVE;
            } else if (strafe < -JOYSTICK_SENSITIVITY || strafe > JOYSTICK_SENSITIVITY){
                driveState = STRAFE;
            } else if (turnRight != 0) {
                driveState = TURN_RIGHT;
            } else if (turnLeft != 0) {
                driveState = TURN_LEFT;
            } else {
                driveState = NONE;
            }
        }

        /**
         * This method can be used on instances of a DriveStateMachine to get the current drive state
         * @return the drive state of the robot
         */
        public int getDriveState() {
            return driveState;
        }

        /**
         * This method calculates the power based on the variable driveState and clips it to
         * the range of -0.6 to 0.6
         * @return the motor power
         */
        public double getPower() {
            if (driveState == DRIVE) {
                if (drive < 0) {
                    return -Range.clip((Math.abs(drive)), -0.7, 0.7);
                    //return drive;
                }
                return Range.clip((Math.abs(drive)), -0.7, 0.7);
                //return Range.clip(drive, -MAX_POWER, MAX_POWER);
            } else if (driveState == STRAFE) {
                if (strafe < 0) {
                    return -Range.clip((Math.abs(strafe)), -0.7, 0.7);
                }
                return Range.clip((Math.abs(strafe)), -0.7, 0.7);
                //return Range.clip(strafe, -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_RIGHT) {
                return Range.clip((turnRight), -0.7, 0.7);
                //return Range.clip(turnRight, -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_LEFT) {
                return Range.clip((turnLeft), -0.7, 0.7);
                //return Range.clip(turnLeft, -MAX_POWER, MAX_POWER);
            } else {
                return 0;
            }
        }

        /**
         * This method exponentially calculates power from the input give
         * Exponential drive code allows the driver to make more precise movements
         * @param input the joystick input
         * @return the power that will be applied to the motors
         */
        public double exponential(double input) {
            // exponential 1
          /*
          // TODO write a method to calculate the following constant
          double pow = input / 2.48; // this value is manually calculated to max out power at 0.6 when input is 0.8
          for (double i = 0; i < input; i += 0.05) {
              pow *= 1.04;
          }
          return pow;
          */
            // exponential 2
          /*
          double pow = input / 4.567923525;
          for (double i = 0; i < input; i += 0.05) {
              pow *= 1.08;
          }
          return pow;


           // exponential 3


           double pow = input / 2.13;
           for (double i = 0; i < input; i += 0.05) {
               pow *= 1.02; // change this value, 1.0265
           }
           return pow;


           // exponential 4
          /*
          double pow = input / 1.82;
          for (double i = 0; i < input; i += 0.05) {
              pow *= 1.02;
          }
          return pow;
          */

            double s = 0.9;
            input += 0.1;
            double previousPower = input;
            //telemetry.addData("power before", pow);
            //telemetry.update();

            pow = (s * previousPower + ((1 - s) * previousPower)) / 2;
            telemetry.addData("power after", pow);
            telemetry.update();
            return pow;
        }
    }
}





