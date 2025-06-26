package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.Servo;

public class DarienServoFunctions {
    protected double min = 0.15;
    protected double max = 0.82;
    protected double currentPosition = min;
    protected Servo servo;

    public DarienServoFunctions( Servo theServo ) {
        // Class Constructor
        this.servo = theServo;
    }

    public void setTrackablePosition(double min, double max, double targetPosition) {
        if (min <= targetPosition && targetPosition <= max) {
            servo.setPosition(targetPosition);
            currentPosition = targetPosition;
        }
    }

    public double getTrackablePosition() {
        return currentPosition;
    }
}
