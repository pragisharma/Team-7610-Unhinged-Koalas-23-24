package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helper.MecanumDrivetrain;

@TeleOp(name = "Mecanum Drive Test")
public class MecanumDriveTest extends LinearOpMode {
    DcMotorEx fl, fr, bl, br;
    MecanumDrivetrain chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.get(DcMotorEx.class, "topRight");
        fl = hardwareMap.get(DcMotorEx.class, "topLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");

        chassis = new MecanumDrivetrain(fl, fr, bl, br);

        waitForStart();
        while(opModeIsActive()) {
            chassis.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            chassis.writeEncoderTelemetry(telemetry);
            telemetry.addLine();
            chassis.writePowerTelemetry(telemetry);
            telemetry.update();
        }
    }
}
