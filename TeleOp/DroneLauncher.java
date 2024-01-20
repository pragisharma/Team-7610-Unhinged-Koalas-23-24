package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "drone")
public class droneLauncher extends LinearOpMode {
    private DcMotor motor;
    private Servo servo;

    boolean pressed = false;

    int stage = 0;
    int timer = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("droneMotor");
        servo = hardwareMap.servo.get( "droneServo");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.back) {
                pressed = true;
            }
            if(pressed) {
                if (stage == 0) {
                    motor.setPower(-1);
                    stage = 1;
                }
                if (stage == 1) {
                    if (timer >= 50) {
                        stage = 2;
                    }
                    timer++;
                    telemetry.addData("timer: ", timer);
                    telemetry.update();
                    sleep(20);
                }
                if (stage == 2) {
                    servo.setPosition(0.3); // 1 goes clockwise
                    telemetry.addData("servo position: ", servo.getPosition());
                    telemetry.update();
                    sleep(500);
                    //  stage = 3;
                }
                if (stage == 3) {
                    servo.setPosition(0.75);
                    break;
                }
            }
            sleep(20);
        }

    }

}
