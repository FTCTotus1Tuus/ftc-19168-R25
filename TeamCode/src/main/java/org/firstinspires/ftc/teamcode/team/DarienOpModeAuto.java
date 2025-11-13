package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//import org.apache.commons.math3.geometry.euclidean.twod.Line;


@Config
public class DarienOpModeAuto extends DarienOpMode {

    //    public WebcamName webcam1;
    public VisionPortal visionPortal;
    public ImageProcess imageProcess;
    public AprilTagProcessor aprilTag;
    public static double movement_igain = 0;
    public static double movement_pgain = 0.06;
    public static double distanceToSlowdown = 4; //Inches
    public static double slowdownPower = 0.35;
    //public double[] tiltMotorPID = {0, 0, 0, 0};
    //public double[] slideMotorPID = {0, 0, 0, 0};

    public static double normalPower = 0.3;
    public static double verticalSlidePower = 1; //swapped to 1 from 0.8 needs testing
    public static double strafingInefficiencyFactor = 1.145;

    public double movementStartTime;

    public Pose2D currentRobotPos;

    //vertical slide positions
    public static int barBelow2Pos;
    public static int barPlace2Pos;
    public static int barBelow1Pos;


    FtcDashboard dashboard;


    //encoder movement targets
    public double targetPosX = 0;
    public double targetPosY = 0;
    public double targetPosH = 0;
    public double rotConst = 1;
    public double acceptableXYError = 0.25; //how many inches off the xy movement can be - does not compound
    public static double minimumXYspeed = 5;
    public double currentMovementPower = 0;
    public static double ProportionalCoefficient = 0.3;

    public double currentHeadingDegrees = 0;

    //public double specimenWristUp = 0.48;

    public double currentPosition = 0;
    public static double rotationEncoderConstant = 557;
    //public MotorHelper tiltMotorHelper;
    //public MotorHelper slideMotorHelper;

    @Override
    public void initControls() {
        super.initControls();
        //tiltMotorHelper = new MotorHelper(telemetry);
        //slideMotorHelper = new MotorHelper(telemetry);
        //    webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // true = blue false = red
        //  imageProcess = new ImageProcess();

        // Create the vision portal by using a builder.
        // VisionPortal.Builder builder = new VisionPortal.Builder();

        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // builder.addProcessor(imageProcess);

        // Build the Vision Portal, using the above settings.
        //  visionPortal = builder.build();

        // reverse motors 2 and 3
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);


        odo.resetPosAndIMU();

        dashboard = FtcDashboard.getInstance();
        initAprilTag();


    }

    public void moveToPosition(double globalX, double globalY, double power) {
        moveToPosition(globalX, globalY, currentHeadingDegrees, power);
    }

    /**
     * Moves the robot to a target robot-centric position using an odometry sensor, with positive X moving forward and positive Y strafing to the left.
     *
     * @param globalX Target X position on the field (positive = forward)
     * @param globalY Target Y position on the field (positive = left)
     * @param globalH Target robot heading in degrees.
     * @param power   Target power value in the range -1 to 1
     */
    public void moveToPosition(double globalX, double globalY, double globalH, double power) {
        // uses odometry sensor (not motor encoders) to move by setting robot motor power

        updatePosition(); //VERY NECESSARY WHENEVER THE ROBOT IS MOVING

        movementStartTime = this.time;

        double errorX = globalX - getXPos();
        double errorY = globalY - getYPos();
        double errorH = getErrorRot(globalH); // degrees

        currentMovementPower = power;
        targetPosY = globalY;
        targetPosX = globalX;
        targetPosH = globalH;

        double headingRad = Math.toRadians((getRawHeading()));
        double errorXp = (errorX * Math.cos(headingRad)) - (errorY * Math.sin(headingRad));
        double errorYp = (errorX * Math.sin(headingRad)) + (errorY * Math.cos(headingRad));

        if (Math.abs(errorH) <= 5) {// attempts to make sure jitters happen less
            // TODO: Evaluate if resetting the heading error (errorH) to zero is introducing a compounding heading drift.
            errorH = 0; // if error is within 5 degrees on either side we say we're good
        }
        double errorHNormalized = errorH / 180.0; // Normalize the degree heading error into [-1,1] as a turn power command
        setPower(power, errorXp, errorYp, errorHNormalized); // add pid?

    }

    public void moveXY(double x, double y, double power) {
        resetEncoder();

        int adjY = (int) Math.floor((y * inchesToEncoder + 0.5));
        int adjX = (int) Math.floor((x * inchesToEncoder * strafingInefficiencyFactor + 0.5));

        omniMotor0.setTargetPosition(adjY + adjX);
        omniMotor1.setTargetPosition(adjY - adjX);
        omniMotor2.setTargetPosition(adjY - adjX);
        omniMotor3.setTargetPosition(adjY + adjX);

        telemetry.addData("omnimotor 0: ", adjY + adjX);
        telemetry.addData("omnimotor 1: ", adjY - adjX);
        telemetry.addData("omnimotor 2: ", adjY - adjX);
        print("omnimotor 3: ", adjY + adjX);
        setBreakpoint();

        setRunMode();
        setPower(power, adjX, adjY, 0);
    }

    public void encoderRotate(double targetPosRadians, double power, boolean rotateClockwise) {
        // rotates to relative position
        resetEncoder();

        int errorBig = (int) (targetPosRadians * rotationEncoderConstant);

        omniMotor0.setTargetPosition(errorBig * (rotateClockwise ? 1 : -1));
        omniMotor1.setTargetPosition(-errorBig * (rotateClockwise ? 1 : -1));
        omniMotor2.setTargetPosition(errorBig * (rotateClockwise ? 1 : -1));
        omniMotor3.setTargetPosition(-errorBig * (rotateClockwise ? 1 : -1));

        setRunMode();

        setRotateEncoderPower(power, rotateClockwise ? 1 : -1);

        currentPosition = targetPosRadians;

    }

    public void autoRotate(double targetPosDegrees, double power) {
        //direction counter clockwise is -1 clockwise is 1

        double error = getErrorRot(targetPosDegrees);
        boolean isRotating = true;
        double direction = Math.signum(error);
        setRotatePower(power, direction);

        if (Math.abs(error) <= rotationTolerance) {
            print("no rotate needed", "");
            return;
        }
        while (isRotating) {
            updatePosition(); //VERY NESSCESSARY WHENEVER THE ROBOT IS MOVING

            error = getErrorRot(targetPosDegrees);

            if (Math.abs(error) <= rotationTolerance) {
                isRotating = false;
            } else if (Math.abs(error) <= rotationTolerance * 5) {
                power /= 3;
            }

            direction = Math.signum(error);
            setRotatePower(power, direction);
        }
        telemetry.addData("rotate end", "");
        telemetry.update();
        setRotatePower(0, 0);
        currentHeadingDegrees = targetPosDegrees; // updates global heading so we can realign after each movment

    }

    public void autoRotate(double targetPosDegrees, double power, boolean isEncoder) {
        setToRotateRunMode();
        autoRotate(targetPosDegrees, power);
        resetEncoder();

    }


    public double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2 / (1 + Math.pow(2.71, (-4 * x)))) - 1;
    }

    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }


    public void setRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setToRotateRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets power to each holonomic wheel, assuming motor order 0 to 3 is a Z configuration (front-left, front-right, rear-left, rear-right).
     *
     * @param power Target motor power (-1 to 1)
     * @param adjX  Target x position (positive = forward)
     * @param adjY  Target y position (positive = left strafe)
     * @param adjH  Target turn rate command (-1 to 1), where positive is CCW and negative is CW
     * @return Scaled motor powers
     */
    public double[] setPower(double power, double adjX, double adjY, double adjH) {

        double[] motorPowers = scalePower(
                (+adjX - adjY - adjH * rotConst),
                (+adjX + adjY + adjH * rotConst),
                (+adjX + adjY - adjH * rotConst),
                (+adjX - adjY + adjH * rotConst), power);

        omniMotor0.setPower(relativePower(motorPowers[0]));
        omniMotor1.setPower(relativePower(motorPowers[1]));
        omniMotor2.setPower(relativePower(motorPowers[2]));
        omniMotor3.setPower(relativePower(motorPowers[3]));

        return motorPowers;
    }

    public double[] scalePower(double motorPower0, double motorPower1, double motorPower2, double motorPower3, double power) {
        double maxPower = Math.max(Math.max(Math.abs(motorPower0), Math.abs(motorPower1)), Math.max(Math.abs(motorPower2), Math.abs(motorPower3)));
        if (maxPower > power) {
            motorPower0 = (motorPower0 * power) / maxPower;
            motorPower1 = (motorPower1 * power) / maxPower;
            motorPower2 = (motorPower2 * power) / maxPower;
            motorPower3 = (motorPower3 * power) / maxPower;
        }

        double[] returnPower = new double[]{
                motorPower0, motorPower1, motorPower2, motorPower3
        };
        return returnPower;
    }


    public void setRotatePower(double power, double direction) {
        omniMotor0.setPower(relativePower(-direction * power));
        omniMotor1.setPower(relativePower(direction * power));
        omniMotor2.setPower(relativePower(-direction * power));
        omniMotor3.setPower(relativePower(direction * power));
    }

    public void setRotateEncoderPower(double power, double direction) {
        omniMotor0.setPower(relativePower(direction * power));
        omniMotor1.setPower(relativePower(-direction * power));
        omniMotor2.setPower(relativePower(direction * power));
        omniMotor3.setPower(relativePower(-direction * power));
    }


    public void resetEncoder() {
        omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetEncoderNoStop() {
        //WARNING: this uses a deprecated mode (RESET_ENCODERS) may not work as intended
        omniMotor0.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void waitForMotors(double timeout, boolean noPID, double errorBand, boolean noSlowdown) {
        boolean looping = true;
        double errorX = 0;
        double errorY = 0;
        double errorXp;
        double errorYp;
        double errorH = 0; // degrees
        double errorHNormalized = 0;
        double errorHrads;
        double headingRad;
        double[] motorPowers;

        if (errorBand == 0) {
            errorBand = acceptableXYError;
        }

        //PID commands
        double movement_pduty = 0;
        double movement_iduty = 0;
        double movement_power;

        Pose2D velocity;


        while (looping) {
            updatePosition(); // VERY NESSCESSARY WHENEVER WE ARE MOVING

            telemetry.addData("x: ", errorX);
            telemetry.addData("y: ", errorY);
            print("h (deg): ", getRawHeading());
            errorX = targetPosX - getXPos();
            errorY = targetPosY - getYPos();
            errorH = getErrorRot(targetPosH);
            errorHNormalized = errorH / 180.0; // Normalize the degree heading error into [-1,1] as a turn power command

            errorHrads = Math.toRadians(errorH);

            if (Math.abs(errorH) <= 5) {// attempts to make sure jitters happen less
                errorH = 0; // if error is within 5 degrees on either side we say we're good
            }

            headingRad = Math.toRadians((getRawHeading()));
            errorXp = (errorX * Math.cos(headingRad)) - (errorY * Math.sin(headingRad));
            errorYp = (errorX * Math.sin(headingRad)) + (errorY * Math.cos(headingRad));


            if (noPID) {
                if (getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                    motorPowers = setPower(slowdownPower, errorXp, errorYp, errorHNormalized); // add pid?
                    telemetry.addData("slow speed - no pid", "");
                } else {
                    //this code works 9/20/2025
                    motorPowers = setPower(currentMovementPower, errorXp, errorYp, errorHNormalized); // add pid?
                    telemetry.addData("full speed - no pid", "");
                }
            } else {
                if (getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                    motorPowers = setPower(slowdownPower, errorXp, errorYp, errorHNormalized); // add pid?
                    telemetry.addData("final approach - pid", "");
                } else {

                    movement_pduty = clamp(movement_pgain * Math.pow(getHypotenuse(errorXp, errorYp), 3 / 2), -1, 1);
                    movement_iduty = clamp(movement_igain * (getHypotenuse(errorXp, errorYp)) + movement_iduty, -.7, .7);
                    movement_power = clamp(movement_pduty + movement_iduty, -currentMovementPower, currentMovementPower);
                    motorPowers = setPower(movement_power, errorXp, errorYp, errorHNormalized);
                    telemetry.addData("current move power: ", movement_power);
                }
            }
            //exit controls
            if (getHypotenuse(errorX, errorY) <= errorBand) {
                looping = false;
            } else if (getHypotenuse(odo.getVelX(), odo.getVelY()) <= minimumXYspeed &&
                    getHypotenuse(errorX, errorY) < acceptableXYError * 4) {
                looping = false;
            } else if ((this.time - movementStartTime) > timeout) {
                looping = false;
            }
            //DASHBOARD TELEMETRY

            velocity = odo.getVelocity();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x pos", getXPos());
            packet.put("y pos", getYPos());
            packet.put("h pos", getRawHeading());
            packet.put("h errorH", errorH);
            packet.put("h errorHNormalized", errorHNormalized);
            packet.put("h odo", Math.toDegrees(odo.getHeading()));
            packet.put("x vel", velocity.getY(DistanceUnit.INCH));
            packet.put("y vel", velocity.getX(DistanceUnit.INCH));
            packet.put("errorX", errorX);
            packet.put("errorY", errorY);
            packet.put("errorXp", errorXp);
            packet.put("errorYp", errorYp);
            packet.put("Motor powers", motorPowers);

            dashboard.sendTelemetryPacket(packet);


        }

        currentHeadingDegrees = getRawHeading();
        setPower(0, 0, 0, 0);

        telemetry.addData("x pos: ", getXPos());
        print("y pos: ", getYPos());

    }

    public void waitForMotors(double timeout, boolean noPid, double errorBand) {
        waitForMotors(timeout, noPid, errorBand, false);
    }

    public void waitForMotors(double timeout, boolean noPID) {
        waitForMotors(timeout, noPID, 0);
    }

    public void waitForMotors(double timeout) {
        waitForMotors(timeout, false, 0);
    }

    public void waitForMotors() {
        waitForMotors(4, false, 0);
    }

    public void waitForMotors(boolean usingJustEncoders) {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()) {
        }
    }

    public void setBreakpoint() {
        while (!gamepad1.a) {
        }
    }

    public double getProportionalSlowdown(double errorX, double errorY) {
        return getHypotenuse(errorX, errorY) * ProportionalCoefficient;
    }

    /**
     * Computes the smallest angular error (difference) between the robot’s current heading and a target heading, in degrees.
     * It ensures the result is always in the range [-180°, 180°], which makes it easy to know whether you should turn clockwise (positive) or counterclockwise (negative).
     *
     * @param targetPosRot Target heading in degrees [-360, 360]
     * @return Number of degrees to turn [-180, 180] to reach target heading (positive is clockwise)
     */
    public double getErrorRot(double targetPosRot) {
        return (((targetPosRot - getRawHeading()) + 180) % 360 + 360) % 360 - 180;
    }

    public void updatePosition() {
        odo.update();
        currentRobotPos = odo.getPosition();
    }

    /**
     * This gets the heading in degrees. Be aware that this normalizes the angle to be between -180 and 180 for DEGREES.
     *
     * @return number of degrees [-180, 180]
     */
    public double getRawHeading() {
        return currentRobotPos.getHeading(AngleUnit.DEGREES);
    }

    public double getXPos() {
        return currentRobotPos.getX(DistanceUnit.INCH);
    }

    public double getYPos() {
        return currentRobotPos.getY(DistanceUnit.INCH);
    }

    @Override
    public void print(String Name, Object message) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        telemetry.update();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        /*
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        */
    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


//    public double getRawHeading() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.h;
//    }
//
//    public double getXPos() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.x;
//    }
//
//    public double getYPos() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.y;
//    }

}
