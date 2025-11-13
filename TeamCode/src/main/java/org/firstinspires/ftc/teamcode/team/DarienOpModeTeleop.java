package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;
    public static double rtvoltage = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update(); // Send telemetry to the driver controller only here.
    }

    public void runClaw() {
        if (gamepad2.right_trigger > rtvoltage) {
            //if right trigger is pressed, claw is opened depending on how much the trigger is pressed
            claw.setPosition((OPEN_CLAW - CLOSED_CLAW) * gamepad2.right_trigger + CLOSED_CLAW);

        } else {
            //if right trigger is not pressed, claw is closed
            claw.setPosition(CLOSED_CLAW);
        }
    }


    /**
     * If the GP1 left bumper is pressed, spin the boot wheels in if the joystick is pulled down or spin the boot wheels out if the joystick is pushed up.
     */

    public void runSlideMotorSystem() {
        telemetry.addData("y=", gamepad2.right_stick_y);
        telemetry.addData("encodervalue=", slideMotor1.getCurrentPosition());

        double power = -0.6;
        double turbo = (1 - power) * gamepad2.right_trigger; // the sum of turbo + power should max at 1.
        //slideMotor1.setPower(gamepad2.right_stick_y * (power + turbo));
        if ((gamepad2.right_stick_y < 0 && slideMotor1.getCurrentPosition() <= 2200) ||
                (gamepad2.right_stick_y > 0 && slideMotor1.getCurrentPosition() >= 100)) {
            // SAFE ZONE NORMAL POWER
            slideMotor1.setPower(gamepad2.right_stick_y * (power));
        } else {
            if ((gamepad2.right_stick_y < 0 && slideMotor1.getCurrentPosition() <= 2300) ||
                    (gamepad2.right_stick_y > 0 && slideMotor1.getCurrentPosition() >= 0)) {
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


    /**
     * runMotorWithEncoderStops
     * This is a generic function to run any DcMotor using encoder stops at min and max encoder values.
     *
     * @param motor                   the motor to be controlled with its encoder
     * @param driverInput             the value from the gamepad control, such as gamepad2.right_stick_y
     * @param defaultMaxPower         the absolute value of the maximum power value, in the range 0-1
     * @param encoderMin              the lower limit of the encoder value, such as 0;
     * @param encoderMinStartSlowdown the number of encoder ticks from which to slow down the motor when approaching the lower limit, such as 100.
     * @param encoderMax              the upper limit of the encoder value, such as 2300;
     * @param encoderMaxStartSlowdown the number of encoder ticks from which to slow down the motor when approaching the upper limit, such as 100.
     */
    public void runMotorWithEncoderStops(DcMotor motor, float driverInput, String MotorName, double defaultMaxPower, int encoderMin, int encoderMinStartSlowdown, int encoderMax, int encoderMaxStartSlowdown) {
        telemetry.addData(MotorName.concat(" Driver Input"), driverInput);
        telemetry.addData(MotorName.concat(" Current Position"), motor.getCurrentPosition());
        telemetry.addData("stop", 0);
        int minTolerance = 10, maxTolerance = 0;

        // Ensure input power is between 0 and 1.
        double power = (defaultMaxPower > 1 || defaultMaxPower < 0) ? 1 : defaultMaxPower;

        // Use negative power value. Assumes the motor is in REVERSE drive mode.
        if (motor.getDirection() == DcMotor.Direction.REVERSE) {
            power = -power;
        }

        if ((driverInput < 0 && motor.getCurrentPosition() <= (encoderMax - encoderMaxStartSlowdown)) ||
                (driverInput > 0 && motor.getCurrentPosition() >= encoderMin + encoderMinStartSlowdown)) {
            // SAFE ZONE NORMAL POWER
            motor.setPower(driverInput * (power));
        } else {
            if ((driverInput < 0 && motor.getCurrentPosition() <= encoderMax - maxTolerance) ||
                    (driverInput > 0 && motor.getCurrentPosition() >= encoderMin + minTolerance)) {
                // DANGER ZONE HALF POWER
                motor.setPower(driverInput * (0.5) * (power));
            } else {
                // END ZONE STOP
                motor.setPower(0);
            }
        }
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