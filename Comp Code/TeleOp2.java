package org.firstinspires.ftc.teamcode.Auto.Final;

import static org.firstinspires.ftc.teamcode.Auto.Final.TeleOp2.ArmStates.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This class describes a Teleop that contains a 20 millisecond loop controlling
 * the 20:1 Core Hex Motors used for the wheels and linear slide based on input from a gamepad
 */
@TeleOp(name = "Saratoga Qualifier Teleop")
public class TeleOp2 extends LinearOpMode {
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


    //ARM STUFF HERE
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, L1_DEPLOY_HIGH, L2_PICKUP, PICKUP, L1_RETRACT_HIGH, L2_STORAGE, //yeah this part's a mess
        FIRST_LINE, SECOND_LINE, THIRD_LINE,
        LINE_RETRACT}
    TeleOp2.ArmStates armState = STORAGE;

    double STORAGE_ALPHA_ANGLE; // set when OpMode is initialized
    int STORAGE_BETA_TICKS = 0;

    double PICKUP_ALPHA_ANGLE;
    double PICKUP_SECONDARY_ALPHA; // the one you lift to for joint 2 to have clearance
    int PICKUP_BETA_TICKS = 1700;

    double FIRST_ALPHA_ANGLE;
    int FIRST_BETA_TICKS = 403;

    double SECOND_ALPHA_ANGLE;
    int SECOND_BETA_TICKS = 574;

    double THIRD_ALPHA_ANGLE;
    int THIRD_BETA_TICKS = 1095;

    //arm control constants
    //final double aP = 0.01; //unused
    final double bP = 0.005;
    double maxArmPower = 1.0;

    //drone stuff
    DcMotor droneMotor;
    Servo droneServo;

    boolean droneIsPressed = false;

    int droneStage = 0;
    int droneTimer = 0;

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

        // set brake and direction (power is intuitive after direction reverse)
        // all + power => robot moves forward
        driveSM.setInitialBehavior();
        //gripper.setPosition(0);

        //MORE ARM STUFF
        joint1 = hardwareMap.get(DcMotor.class, "joint 1");
        pot = hardwareMap.get(AnalogInput.class, "joint 1 pot");
        STORAGE_ALPHA_ANGLE = (pot.getVoltage() * 270 / 3.3);

        joint2 = hardwareMap.get(DcMotor.class, "joint 2");
        claw = hardwareMap.get(Servo.class, "claw");

        //init all arm alphas since putting them in init would set STORAGE_ALPHA_ANGLE to 0 >:(
        PICKUP_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 30.6;
        PICKUP_SECONDARY_ALPHA = STORAGE_ALPHA_ANGLE + 35;
        FIRST_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 67;
        SECOND_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 68;
        THIRD_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 95;

        //reset encoders to 0
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //no moving
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //drone init
        droneMotor = hardwareMap.dcMotor.get("droneMotor");
        droneServo = hardwareMap.servo.get( "droneServo");

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

            arm();
            claw();
            drone();

            telemetry.addData("pot voltage", pot.getVoltage());
            telemetry.addData("pot angle", pot.getVoltage()/3.3 * 270);
            telemetry.addData("joint 2 ticks", joint2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("joint 1 power", joint1.getPower());
            telemetry.addData("joint 2 power", joint2.getPower());
            telemetry.addLine();
            telemetry.addData("claw position, 1 = open", claw.getPosition());

            writeTelemetry();
            telemetry.update();
            sleep(20);

        }
        //slide.setTargetPosition(armSM.BASE);
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //gripper.setPosition(0);

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
        //open when droneIsPressed
        if (gamepad2.right_bumper) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
    }

    /*-- arm helpers --*/
    public double setJoint1Power(double alpha, double target) {
        if (aClose(alpha, target)) {
            return 0;
            //a little fudge since it would do the flop
        } else {
            return -maxArmPower * (target-alpha)/alpha;
        }
    }

    public boolean aClose(double actual, double target) {
        return Math.abs(target - actual) <= 1;
    }

    public boolean bClose(double actual, double target) {
        return Math.abs(target - actual) <= 10;
    }

    public void drone() {
        if(gamepad1.back) {
            droneIsPressed = true;
        }
        if(droneIsPressed) {
            if (droneStage == 0) {
                droneMotor.setPower(-1);
                droneStage = 1;
            }
            if (droneStage == 1) {
                if (droneTimer >= 50) {
                    droneStage = 2;
                }
                droneTimer++;
                telemetry.addData("drone droneTimer: ", droneTimer);
            }
            if (droneStage == 2) {
                droneServo.setPosition(0.3); // 1 goes clockwise
                telemetry.addData("drone servo position: ", droneServo.getPosition());
                telemetry.update();
                droneTimer++;
                if(droneTimer >= 100) {
                    droneStage = 3;
                }
            }
            if (droneStage == 3) {
                droneServo.setPosition(0.55);
                droneMotor.setPower(0);
                if(droneTimer >= 360) {
                    return; //does nothing
                }
            }
        }
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
        private double MAX_POWER = 1.0;
        private static final int STRAFE = 0;
        private static final int DRIVE = 1;
        private static final int TURN_RIGHT = 2;
        private static final int TURN_LEFT = 3;
        private static final int NONE = 4;
        private final double JOYSTICK_SENSITIVITY = 0.0;

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


            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
         * the range of -MAX_POWER to MAX_POWER
         * @return the motor power
         */
        public double getPower() {
            if (driveState == DRIVE) {
                if (drive < 0) {
                    return -Range.clip((Math.abs(scale(drive))), -MAX_POWER, MAX_POWER);
                }
                return Range.clip(drive, -MAX_POWER, MAX_POWER);
            } else if (driveState == STRAFE) {
                if (strafe < 0) {
                    return -Range.clip((Math.abs(scale(strafe))), -MAX_POWER, MAX_POWER);
                }
                return Range.clip(scale(strafe), -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_RIGHT) {
                return Range.clip(0.5*scale(turnRight), -MAX_POWER, MAX_POWER);
            } else if (driveState == TURN_LEFT) {
                return Range.clip(0.5*scale(turnLeft), -MAX_POWER, MAX_POWER);
            } else {
                return 0;
            }
        }

        /**
         * This method exponentially calculates power from the input given
         * Exponential drive code allows the driver to make more precise movements
         * @param x the joystick input
         * @return the power that will be applied to the motors
         */
        public double scale(double x) {
            return Math.signum(x) * x*x;
        }
    }
}




