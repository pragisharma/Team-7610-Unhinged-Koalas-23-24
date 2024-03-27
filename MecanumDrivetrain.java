package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;

//just basic implementation, nothing fancy. Drives, strafes, and turns. PID as well :D
public class MecanumDrivetrain {
    private final DcMotor tl, tr, bl, br;
    private PIDController pid;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public MecanumDrivetrain(DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br) {
        this.tl = tl;
        this.tr = tr;
        this.bl = bl;
        this.br = br;

        tr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public MecanumDrivetrain(DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br, PIDController pid) {
        this.tl = tl;
        this.tr = tr;
        this.bl = bl;
        this.br = br;
        this.pid = pid;

        tr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        //since we have a custom PID now
        tl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void writeEncoderTelemetry(Telemetry telemetry) {
        telemetry.addData("Front Left ticks", tl.getCurrentPosition());
        telemetry.addData("Front Right ticks", tr.getCurrentPosition());
        telemetry.addData("Back Left ticks", bl.getCurrentPosition());
        telemetry.addData("Back Right ticks", br.getCurrentPosition());
    }

    public void writePowerTelemetry(Telemetry telemetry) {
        telemetry.addData("Front Left power", tl.getPower());
        telemetry.addData("Front Right power", tr.getPower());
        telemetry.addData("Back Left power", bl.getPower());
        telemetry.addData("Back Right power", br.getPower());
    }

    public PIDController getPid() {
        return pid;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public void move(double drive1, double strafe1, double turn1) {
        double drive = drive1;
        double strafe = -1.3 * strafe1;
        double turn = -turn1;

        tl.setPower(drive + strafe + turn);
        tr.setPower(drive - strafe - turn);
        bl.setPower(drive - strafe + turn);
        br.setPower(drive + strafe - turn);
    }

    //default is forward
    public void drive(double power) {
        if (pid == null) {
            tl.setPower(power);
            tr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        } else {
            double target = pid.update(power, tl.getPower());
            tl.setPower(target);
            tr.setPower(target);
            bl.setPower(target);
            br.setPower(target);
        }
    }

    //default is right
    public void strafe(double power) {
        if (pid == null) {
            tl.setPower(power);
            tr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
        } else {
            double target = pid.update(power, tl.getPower());
            tl.setPower(target);
            tr.setPower(-target);
            bl.setPower(-target);
            br.setPower(target);
        }
    }

    //default is CW (right)
    public void turn (double power) {
        if (pid == null) {
            tl.setPower(power);
            tr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        } else {
            double target = pid.update(power, tl.getPower());
            tl.setPower(target);
            tr.setPower(-target);
            bl.setPower(target);
            br.setPower(-target);
        }
    }

    public void stop() {
        if (pid == null) {
            tl.setPower(0);
            tr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        } else {
            double target = pid.update(0, tl.getPower());
            tl.setPower(target);
            tr.setPower(target);
            bl.setPower(target);
            br.setPower(target);
        }
    }

    public void resetEncoders() {
        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
