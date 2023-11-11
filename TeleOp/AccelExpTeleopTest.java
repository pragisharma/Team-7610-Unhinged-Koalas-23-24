package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Acceleration and Exponential Power Test")
public class AccelExpTeleopTest extends LinearOpMode {
    DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotor.class, "topRight");
        fl = hardwareMap.get(DcMotor.class, "topLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //definitely didn't forget
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        double power;

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {
                //forward/backwards
                power = gamepad1.left_stick_y;
                power = accel(fl.getPower(), expScale(power));
                fl.setPower(-power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(-power);

                telemetry.addLine("forwards/backwards");
            } else if (gamepad1.left_trigger > 0) {
                //strafe left
                power = gamepad1.left_trigger;
                power = accel(fl.getPower(), expScale(power));
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);

                telemetry.addLine("strafe left");
            } else if (gamepad1.right_trigger > 0) {
                //strafe right
                power = gamepad1.right_trigger;
                power = accel(fl.getPower(), expScale(power));
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);

                telemetry.addLine("strafe right");
            } else if (gamepad1.right_stick_x != 0) {
                //turn
                power = gamepad1.right_stick_x;
                power = accel(fl.getPower(), expScale(power));
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(power);
                br.setPower(-power);

                telemetry.addLine("turn");
            } else {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);

                telemetry.addLine("stopped");
            }

            telemetry.addData("Front Left Power", fl.getPower());
            telemetry.addData("Front Right Power", fr.getPower());
            telemetry.addData("Back Left Power", bl.getPower());
            telemetry.addData("Back Right Power", br.getPower());

            telemetry.update();

            sleep(20);
        }
    }

    public double expScale(double pow) {
        return Math.pow(1.8, pow) - 1;
    }

    public double accel(double pow, double target) {
        return 0.1 * pow + 0.9 * target;
    }
}
