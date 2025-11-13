package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "MyFirstAuto", group = "State")
public class MyFirstAuto extends DarienOpModeAuto {

    private MotorHelper tiltMotorHelper;
    private MotorHelper slideMotorHelper;
    private double tilt_gain = 1;
    private double T2 = 1000;
    private float tiltSetPoint = 0;
    private float slideSetPoint = 0;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHDegrees = 0;
    public static boolean noPid = true;
    public static boolean noSlowdown = true;

    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        double[] tiltMotorPID = {0, 0, 0};
        double[] slideMotorPID = {0, 0, 0};
        tiltMotorHelper = new MotorHelper(telemetry);
        slideMotorHelper = new MotorHelper(telemetry);

        while (this.opModeIsActive()) {
            // Goal: Drive to test our odometry tuning and test all mechanisms
            //Move forward back left right, extend slide and raise it, then drop the slide and retract it

            // updatePosition is needed before reading the x,y positions from odometry.
            updatePosition();

            moveToPosition(targetX, targetY, targetHDegrees, .4);
            waitForMotors(4, noPid, 0, noSlowdown);
            setBreakpoint();

            /*List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                switch (detection.id) {
                    case 21:
                        telemetry.addData("Motif", "GPP");
                        break;
                    case 22:
                        telemetry.addData("Motif", "PGP");
                        break;
                    case 23:
                        telemetry.addData("Motif", "PPG");
                        break;
                    case 24:
                        telemetry.addData("Goal", "Red");
                        break;
                    case 20:
                        telemetry.addData("Goal", "Blue");
                        break;
                }
            }

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 23) {
                    moveToPosition(targetX, targetY, .2);
                    waitForMotors(4, false, 0, true);
                } else if (detection.id == 24) {
                    moveToPosition(targetX, targetY, .2);
                    waitForMotors(4, false, 0, true);
                }

            }
             */
            telemetry.addData("x pos: ", getXPos());
            print("y pos: ", getYPos());

            /*
            moveToPosition(5, 0, 0.2);
            waitForMotors();
            sleep(250);

            moveToPosition(0, 0, 0.2);
            waitForMotors();
            sleep(250);

            moveToPosition(0, 10, 0.2);
            waitForMotors();
            sleep(250);
            moveToPosition(0, 0, 0.2);
            waitForMotors();
            sleep(250);
            tiltSetPoint = 900;
            waitForMotors();
            sleep(100);
            slideSetPoint = 2500;
            waitForMotors();
            sleep(2500);
            tiltSetPoint = 0;
            waitForMotors();
            sleep(100);
            slideSetPoint = 0;
            waitForMotors();
            moveToPosition(10, 10, 0.2);
            waitForMotors();
            autoRotate(360, 0.2);
            waitForMotors();

             */


           /*
           tiltMotorPID = tiltMotorHelper.pid(
                    tiltMotor,
                    tilt_pgain,
                    tilt_pgain2,
                    tilt_igain,
                    tiltMotorPID[2], -.5, 0.8, -0.2, 1, tiltMotorPID[1], -.5, .25,
                    tilt_gain,
                    T1,
                    T2,
                    tiltSetPoint,
                    false
            );
            tiltMotor.setPower(tiltMotorPID[0]);
            slideMotorPID = slideMotorHelper.pid(
                    slideMotor1,
                    slide_pgain,
                    slide_pgain2,
                    slide_igain,
                    slideMotorPID[2], -.7, .6, -.7, .6, slideMotorPID[1], -1, 1,
                    slide_gain,
                    S1,
                    S2,
                    slideSetPoint,
                    true
            );
            slideMotor1.setPower(slideMotorPID[0]);
            */
            if (gamepad1.back) return;
        }
    }
}