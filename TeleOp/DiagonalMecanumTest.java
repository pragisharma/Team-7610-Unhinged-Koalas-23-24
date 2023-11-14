package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Diagonal Mecanum Test")
public class DiagonalMecanumTest extends LinearOpMode {
    DcMotor fl, fr, bl, br;

    float trig45 = (float)(Math.sqrt(2) / 2.0); //since 45 degree trig is so nice

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

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_stick_x != 0) {
                //turn
                double pow = gamepad1.right_stick_x;
                pow *= 0.8; //lazy, no exp

                fl.setPower(pow);
                fr.setPower(-pow);
                bl.setPower(pow);
                br.setPower(-pow);
            } else {
                Pair<Float, Float> leftJoystickXY = new Pair<>(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                Pair<Float, Float> powerVec = rotateCW(leftJoystickXY);

                fl.setPower(powerVec.first);
                br.setPower(powerVec.first);
                fr.setPower(powerVec.second);
                bl.setPower(powerVec.second);
            }
        }
    }

    public Pair<Float,Float> rotateCW(Pair<Float,Float> gamepadInput) {
        return new Pair<>(
                trig45 * gamepadInput.first + trig45 * gamepadInput.second,
                -trig45 * gamepadInput.first + trig45 * gamepadInput.second
        );
    }

    public Pair<Float,Float> rotateCCW(Pair<Float,Float> gamepadInput) {
        return new Pair<>(
                trig45 * gamepadInput.first + -trig45 * gamepadInput.second,
                trig45 * gamepadInput.first + trig45 * gamepadInput.second
        );
    }
}
