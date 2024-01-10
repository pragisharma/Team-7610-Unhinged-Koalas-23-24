package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.EasyArmButtons.ArmStates.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class MoveToPickup {
    DcMotor joint1, joint2;
    AnalogInput pot;
    Servo claw;

    enum ArmStates {STORAGE, L1_DEPLOY_HIGH, L2_PICKUP, PICKUP, L1_RETRACT_HIGH, L2_STORAGE}
    EasyArmButtons.ArmStates armState = STORAGE;

    double STORAGE_ALPHA_ANGLE; // Make sure to init this when you init the opmode
    //STORAGE_ALPHA_ANGLE = (pot.getVoltage() * 270 / 3.3);
    //^^init code
    int STORAGE_BETA_TICKS = 0;

    double PICKUP_ALPHA_ANGLE = STORAGE_ALPHA_ANGLE + 38;
    double PICKUP_SECONDARY_ALPHA = STORAGE_ALPHA_ANGLE + 43; // the one you lift to for joint 2 to have clearance
    int PICKUP_BETA_TICKS = 1700;

    final double bP = 0.005;
    double maxArmPower = 1.0;

    public void storageToPickup() {
        while (armState != PICKUP) {
            double alpha = (pot.getVoltage() * 270 / 3.3);
            double beta = joint2.getCurrentPosition();

            if (armState == STORAGE) {
                joint1.setPower(0);
                joint2.setPower(0);

                armState = L1_DEPLOY_HIGH;
            } else if (armState == L1_DEPLOY_HIGH) {
                joint1.setPower(setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA));
                joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -1, 1));

                if (aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                    armState = L2_PICKUP;
                }
            } else if (armState == L2_PICKUP) {
                joint1.setPower(setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA));
                joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -1,1));

                if (!aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                    armState = L1_DEPLOY_HIGH;
                } else if (bClose(beta, PICKUP_BETA_TICKS)) {
                    armState = PICKUP;
                }
            }
        }

        if (armState == PICKUP) { //shut up android studio helper im paranoid
            joint1.setPower(0);
            joint2.setPower(0);
        }
    }

    public void pickupToStorage() {
        while (armState != STORAGE) {
            double alpha = (pot.getVoltage() * 270 / 3.3);
            double beta = joint2.getCurrentPosition();

            if (armState == PICKUP) {
                joint1.setPower(0);
                joint2.setPower(0);

                armState = L1_RETRACT_HIGH;
            } else if (armState == L1_RETRACT_HIGH) {
                joint1.setPower(setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA));
                joint2.setPower(Range.clip(bP * (PICKUP_BETA_TICKS - beta), -maxArmPower, maxArmPower));

                if (aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                    armState = L2_STORAGE;
                }
            } else if (armState == L2_STORAGE) {
                joint1.setPower(setJoint1Power(alpha, PICKUP_SECONDARY_ALPHA));
                joint2.setPower(Range.clip(bP * (STORAGE_BETA_TICKS - beta), -maxArmPower, maxArmPower));

                if (!aClose(alpha, PICKUP_SECONDARY_ALPHA)) {
                    armState = L1_RETRACT_HIGH;
                } else if (bClose(beta, STORAGE_BETA_TICKS)) {
                    armState = STORAGE;
                }
            }
        }

        if (armState == STORAGE) { //shut up android studio helper im paranoid
            joint1.setPower(0);
            joint2.setPower(0);
        }
    }

    public double setJoint1Power(double alpha, double target) {
        if (aClose(alpha, target)) {
            return 0;
            //a little fudge since it would do the flop
        } else if (target - alpha > 0) {
            return -1 * Math.abs(target-alpha)/alpha;
        } else {
            return 1 * Math.abs(target-alpha)/alpha;
        }
    }

    public boolean aClose(double actual, double target) {
        return Math.abs(target - actual) <= 1;
    }

    public boolean bClose(double actual, double target) {
        return Math.abs(target - actual) <= 10;
    }
}
