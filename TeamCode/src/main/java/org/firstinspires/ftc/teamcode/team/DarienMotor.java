package org.firstinspires.ftc.teamcode.team;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DarienMotor {
    final DcMotor motor;
    public DarienMotor(final HardwareMap hMap, final String nameInHardware, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor = hMap.get(DcMotor.class, nameInHardware);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

//    public static void main(String[] args) {
//        DarienMotor slideMotor1 = new DarienMotor("s", "slideMotor1", DcMotor.Direction.REVERSE,DcMotor.ZeroPowerBehavior.BRAKE);   );
//    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
    /** runMotorWithEncoderStops
     * This is a generic function to run any DcMotor using encoder stops at min and max encoder values.
     * @param driverInput the value from the gamepad control, such as gamepad2.right_stick_y
     * @param defaultMaxPower the absolute value of the maximum power value, in the range 0-1
     * @param encoderMin the lower limit of the encoder value, such as 0;
     * @param encoderMinStartSlowdown the number of encoder ticks from which to slow down the motor when approaching the lower limit, such as 100.
     * @param encoderMax the upper limit of the encoder value, such as 2300;
     * @param encoderMaxStartSlowdown the number of encoder ticks from which to slow down the motor when approaching the upper limit, such as 100.
     */
    public void runMotorWithEncoderStops(float driverInput, String MotorName, double defaultMaxPower, int encoderMin, int encoderMinStartSlowdown, int encoderMax, int encoderMaxStartSlowdown) {
//        telemetry.addData(MotorName.concat(" Driver Input"), driverInput);
//        telemetry.addData(MotorName.concat(" Current Position"), motor.getCurrentPosition());
//        telemetry.addData("stop",0);
        int minTolerance = 10, maxTolerance = 0;

        // Ensure input power is between 0 and 1.
        double power = (defaultMaxPower > 1 || defaultMaxPower < 0) ? 1 : defaultMaxPower;

        // Use negative power value. Assumes the motor is in REVERSE drive mode.
        if (motor.getDirection() == DcMotor.Direction.REVERSE) {
            power = -power;
        }

        if ((driverInput < 0 && motor.getCurrentPosition() <= (encoderMax - encoderMaxStartSlowdown)) ||
                (driverInput > 0 && motor.getCurrentPosition() >= encoderMin + encoderMinStartSlowdown)){
            // SAFE ZONE NORMAL POWER
            motor.setPower(driverInput * (power));
        } else {
            if ((driverInput < 0 && motor.getCurrentPosition() <= encoderMax - maxTolerance) ||
                    (driverInput > 0 && motor.getCurrentPosition() >= encoderMin + minTolerance)){
                // DANGER ZONE HALF POWER
                motor.setPower(driverInput * (0.5) * (power));
            } else {
                // END ZONE STOP
                motor.setPower(0);
            }
        }
    }
}
