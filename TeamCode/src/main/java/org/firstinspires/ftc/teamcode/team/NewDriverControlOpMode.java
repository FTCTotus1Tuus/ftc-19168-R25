package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    // tuning constants for gobilda 117 rpm motor
    public static double tilt_pgain = .01;
    public static double tilt_igain = .0003;
    public static double tilt_gain = 7;
    public static double T2 = 1750;
    public static double T1 = 10;
    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide
    public static double slide_pgain = .005;
    public static double slide_igain = .0004;
    public static double slide_gain = 15;
    public static double S2 = 3900;
    public static double S1 = 0;
    private MotorHelper tiltMotorHelper;
    private MotorHelper slideMotorHelper;

    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        //Start
        double[] tiltMotorPID = {0, 0, 0, 0};
        double[] slideMotorPID = {0, 0, 0, 0};
        tiltMotorHelper = new MotorHelper(telemetry);
        slideMotorHelper = new MotorHelper(telemetry);

        while (this.opModeIsActive()) {
            runClaw();
            //pollSensors();
            runDriveSystem();
            //runSlideMotorSystem();
            //runMotorWithEncoderStops(slideMotor1, gamepad2.right_stick_y,"Slide",0.6, 0, 100, 3900, 100);
            // runMotorWithEncoderStops(tiltMotor, gamepad2.left_stick_y,"Tilt" ,-1, 0, 100, 2300, 100);
            slideMotorPID = slideMotorHelper.pid(
                    slideMotor1,
                    slide_pgain,
                    slide_igain,
                    slideMotorPID[2], -.7, .6, -.7, .6, slideMotorPID[1], -1, 1,
                    slide_gain,
                    S1,
                    S2,
                    gamepad2.right_stick_y,
                    false
            );
            slideMotor1.setPower(slideMotorPID[0]);

            tiltMotorPID = tiltMotorHelper.pid(
                    tiltMotor,
                    tilt_pgain,
                    tilt_igain,
                    tiltMotorPID[2], -.5, 0.8, -0.2, 1, tiltMotorPID[1], -.5, 1,
                    tilt_gain,
                    T1,
                    T2,
                    gamepad2.left_stick_y,
                    false
            );
            tiltMotor.setPower(tiltMotorPID[0]);

        }
    }
}