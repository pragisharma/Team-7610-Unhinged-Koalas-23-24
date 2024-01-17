package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "drone")
public class droneLancher extends LinearOpMode {
    private DcMotor motor;
    private DcMotor servo;

    boolean pressed = false;

    int stage = 0;
    int timer = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("droneMotor");
        servo = hardwareMap.dcMotor.get("droneServo");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.back == true) {
                pressed = true;
            }
            if(pressed = true) {
                if (stage == 0) {
                    motor.setPower(1);
                    stage = 1;
                }
                if (stage == 1) {
                    if (timer >= 500) {
                        stage = 2;
                    }
                    timer++;
                }
                if (stage == 2) {
                    servo.setPower(1);
                    stage = 3;
                }
                if (stage == 3) {
                    servo.setPower(0);
                    break;
                }
            }
        }
    }

}

