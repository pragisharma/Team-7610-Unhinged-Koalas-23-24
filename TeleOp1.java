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

    //private DcMotor slide = null;

    //private Servo gripper = null;

    /**
     * New instance of a DriveStateMachine that controls the motor power and drive state
     */
    DriveStateMachine driveSM = new DriveStateMachine();
    //ArmStateMachine armSM = new ArmStateMachine();
    //GripperStateMachine gripperSM = new GripperStateMachine();

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
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
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


        // arm state telemetry
        /*String armStateStr;
        if (armState == armSM.GROUND) {
            armStateStr = "ground";
        } else if (armState == armSM.LEVEL_1) {
            armStateStr = "level 1";
        } else if (armState == armSM.LEVEL_2) {
            armStateStr = "level 2";
        } else {
            armStateStr = "none";
        }
        telemetry.addData("arm state", armStateStr);
        telemetry.addData("desired count", armState);
        telemetry.addData("curr count", armSM.getCurrCount());
        */

        // gripper state telemetry
        /*String gripperStateStr;
        if (gripperState == gripperSM.CLOSED) {
            gripperStateStr = "closed";
        } else if (gripperState == gripperSM.OPEN) {
            gripperStateStr = "open";
        } else {
            gripperStateStr = "none";
        }
        telemetry.addData("gripper state", gripperStateStr);
        telemetry.addData("curr count", gripperSM.getPosition());
        */

    }


    /**
     * This class describes a DriveStateMachine with accessible attributes of STRAFE, DRIVE,
     * TURN_RIGHT, TURN_LEFT, and NONE
     */
    class DriveStateMachine {
        private int driveState = 0;
        private double pow = 0.0;
        private double MAX_POWER = 0.6;
        private static final int STRAFE = 0;
        private static final int DRIVE = 1;
        private static final int TURN_RIGHT = 2;
        private static final int TURN_LEFT = 3;
        private static final int NONE = 4;
        private final double JOYSTICK_SENSITIVITY = 0.3;

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
                    return -Range.clip((Math.abs(drive)/3.0), -0.6, 0.6);
                    //return drive;
                }
                return Range.clip((Math.abs(drive)/3.0), -0.6, 0.6);
                //return Range.clip(drive, -MAX_POWER, MAX_POWER);
            } else if (driveState == STRAFE) {
                if (strafe < 0) {
                    return -Range.clip((Math.abs(strafe)/3.0), -0.6, 0.6);
                }
                return Range.clip((Math.abs(strafe)/3.0), -0.6, 0.6);
                //return Range.clip(strafe, -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_RIGHT) {
                return Range.clip((turnRight)/3.0, -0.6, 0.6);
                //return Range.clip(turnRight, -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_LEFT) {
                return Range.clip((turnLeft)/3.0, -0.6, 0.6);
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


    /**
     * This class describes an ArmStateMachine
     */
    /*class ArmStateMachine {
        private boolean pressed = true;
        private int desired = 0;
        private int MARGIN = 10;
        private final int BASE = 0;
        //private final int MAX = 1900;
        private final int GROUND = 171; // getTicks(3.75) // TODO modify values
        private final int BEACON = 250;
        private final int LEVEL_1 = 1050; // getTicks(14) // 400
        private final int LEVEL_2 = 1850; // getTicks(24) // 620
        private final int LEVEL_3 = 2620; // getTicks(35);


        public void setEncoderCount() {
            pressed = true;
            if (gamepad2.a) {
                desired = GROUND;
            } else if (gamepad2.b) {
                desired = LEVEL_1;
            } else if (gamepad2.y) {
                desired = LEVEL_2;
            } else if (gamepad2.x) {
                desired = LEVEL_3;
            } else if (gamepad2.dpad_down){
                telemetry.addLine("reached");
                telemetry.addData("power", slide.getPower());
                telemetry.update();
                desired = BASE;
            } else if (gamepad2.dpad_up) {
                desired = BEACON;
            } else {
                pressed = false;
            }
            slide.setTargetPosition(desired);
        }


        public void run() {
            // resetting local variables according to current desired attributes
            double currPosition = slide.getCurrentPosition();
            double target = slide.getTargetPosition();
            if (pressed) {
                slide.setPower(1);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                setEncoderCount();
            }

            // resetting pressed based on if the motor has reached its target position
            // (desired encoder count)
            // margin is the range of error that the encoder can reach (encoder count will
            // usually never be exactly the desired position)
            if (currPosition - MARGIN < target && currPosition + MARGIN > target) {
                pressed = false;
                slide.setPower(0);
            }
        }

        /**
         * This method calculates the number of ticks needed for the slide to move to
         * inches height
         * @param inches the height of the bottom of the cone when it is
         * @return number of ticks needed to reach height inches
         */
        /*public int getTicks(double inches) {
            return (int)((inches * 70.5) - 130);
        }

        /**
         * This method gets the slide state in integer form representing encoder count
         * @return desired encoder count
         */
        /*public int getSlideState() {
            return desired;
        }

        /**
         * This method returns the current encoder count of the slide Core Hex Motor
         * @return current encoder count of the linear slide
         */
        /*public int getCurrCount() {
            return slide.getCurrentPosition();
        }
    }*/

    /*class GripperStateMachine {
        // 270 / 360 = TICKS PER DEGREE
        // TICKS PER DEGREE * DEGREES (60?) = TOTAL TICKS for open state
        // 60:360 = x:270
        // x = 45
        // 270: 1 = 45: y
        // y = 45 / 270
        public double CLOSED = 0;
        public double OPEN = 45.0/270;
        public double NONE = -100;
        public double MARGIN = 6.0/270; // 8 degrees margin - test
        public double state = CLOSED;


        public void setState() {
            if (gamepad2.right_bumper) {
                state = CLOSED;
            } else if (gamepad2.left_bumper) {
                state = OPEN;
            } else {
                state = NONE;
            }
        }

        /**
         * This method sets the position of the servo based on the current state
         */
        /*public void run() {
            if (state == OPEN) {
                gripper.setPosition(0.65);
            } else if (state == CLOSED) {
                gripper.setPosition(0);
            }

            // resets the state if the gripper has reached its maximum open or close encoder value
            double currPos = gripper.getPosition();
            if (state == OPEN && currPos + MARGIN >= 1) {
                state = NONE;
            } else if (state == CLOSED && currPos - MARGIN <= 0) {
                state = NONE;
            }
        }

        /**
         * This method gets the state of the gripper
         * @return double encoder value of the gripper
         */
        /*public double getState() {
            return state;
        }

        /**
         * This method gets the current encoder count of the gripper
         * @return encoder count
         */
        /*public double getPosition() {
            return gripper.getPosition();
        }

    }*/
}






