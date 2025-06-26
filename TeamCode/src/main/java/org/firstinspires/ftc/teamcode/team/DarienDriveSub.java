package org.firstinspires.ftc.teamcode.team;

import static org.firstinspires.ftc.teamcode.team.DarienOpModeTeleop.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DarienDriveSub {

    private DcMotor omniMotor0, omniMotor1, omniMotor2, omniMotor3;
    HardwareMap hMap;
    Telemetry telemetry;

    /**
     * @param hMap
     * @param telemetry
     * @param motorNames 4
     * @param directions 4
     */
    public DarienDriveSub(HardwareMap hMap, Telemetry telemetry, String[] motorNames, DcMotorSimple.Direction[] directions) {

        this.hMap = hMap;
        this.telemetry = telemetry;

        omniMotor0 = initializeMotor(motorNames[0]);
        omniMotor1 = initializeMotor(motorNames[1]);
        omniMotor2 = initializeMotor(motorNames[2]);
        omniMotor3 = initializeMotor(motorNames[3]);

        omniMotor0.setDirection(directions[0]);
        omniMotor1.setDirection(directions[1]);
        omniMotor2.setDirection(directions[2]);
        omniMotor3.setDirection(directions[3]);

        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }


    public void runDriveSystem(Gamepad gamepad1) {
        double[] direction = new double[2];
        double rotation = 0;
        boolean turboBoost;
        direction[0] = Math.pow(-gamepad1.left_stick_x, 5);
        direction[1] = Math.pow(-gamepad1.left_stick_y, 5);
        if (!gamepad1.left_bumper) {
            rotation = Math.pow(-gamepad1.right_stick_x, 5);
        }
        turboBoost = gamepad1.left_stick_button;
        double divBy = (gamepad1.left_trigger / 2) + 0.5;
        MoveRobot(direction, -rotation, divBy);
    }


    public void MoveRobot(double[] direction, double rotation, double divBy) {

        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);

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

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}
