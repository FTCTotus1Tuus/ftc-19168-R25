package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DarienHelperFunctions {


    public static double getHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static double getHypotenuse(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2 / (1 + Math.pow(2.71, (-4 * x)))) - 1;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));

    }

    public static double getVoltage(HardwareMap hardwareMap) {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public static void print(String Name, Object message, Telemetry telemetry) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        telemetry.update();
    }

    public static double relativePower(double intended_power, HardwareMap hardwareMap) {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage(hardwareMap);
    }


}
