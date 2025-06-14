package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update(); // Send telemetry to the driver controller only here.
    }




    /**
     * If the GP1 left bumper is pressed, spin the boot wheels in if the joystick is pulled down or spin the boot wheels out if the joystick is pushed up.
     */

    public void runSlideMotorSystem() {
        telemetry.addData("y=", gamepad2.right_stick_y);
        telemetry.addData("encodervalue=", slideMotor1.getCurrentPosition());

        double power = -0.6;
        double turbo = (1-power)*gamepad2.right_trigger; // the sum of turbo + power should max at 1.
        //slideMotor1.setPower(gamepad2.right_stick_y * (power + turbo));
        if ((gamepad2.right_stick_y < 0 && slideMotor1.getCurrentPosition() <= 2200) ||
                (gamepad2.right_stick_y > 0 && slideMotor1.getCurrentPosition() >= 100)){
            // SAFE ZONE NORMAL POWER
            slideMotor1.setPower(gamepad2.right_stick_y * (power));
        } else {
            if ((gamepad2.right_stick_y < 0 && slideMotor1.getCurrentPosition() <= 2300) ||
                    (gamepad2.right_stick_y > 0 && slideMotor1.getCurrentPosition() >= 0)){
                // DANGER ZONE HALF POWER
                slideMotor1.setPower(gamepad2.right_stick_y * (0.5) * (power));
            } else {
                // END ZONE STOP
                slideMotor1.setPower(0);
            }
        }


        //A: when right stick y set power to 1
        //B: when encoder reaches certain limit, slow down power
        //
    }



    public void runDriveSystem() {
        direction[0] = Math.pow(-gamepad1.left_stick_x, 5);
        direction[1] = Math.pow(-gamepad1.left_stick_y, 5);
        if (!gamepad1.left_bumper) {
            rotation = Math.pow(-gamepad1.right_stick_x, 5);
        }
        turboBoost = gamepad1.left_stick_button;
        MoveRobot(direction, -rotation, turboBoost);
    }


    public void MoveRobot(double[] direction, double rotation, boolean turboBoost) {

        double divBy;
        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);
//        if (turboBoost) {
//            divBy = turboDivBy;
//        } else {
//            divBy = regularDivBy;
//
//        }

        divBy = (gamepad1.left_trigger / 2) + 0.5;
        telemetry.addData("", wheel0 * divBy);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
    }


    public void MoveMotor(DcMotor motor, double power) {
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

}