package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class DriverControl extends DarienOpModeTeleop {
    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        //Start
        while (this.opModeIsActive()) {

            //pollSensors();
            runDriveSystem();
            //runSlideMotorSystem();
            runMotorWithEncoderStops(slideMotor1, gamepad2.right_stick_y, 0.6, 0, 100, 2300, 100);
            runLiftSystem();
            telemetry.update();

            if (gamepad1.a){
                double[] robotPower = {0,0.1};
                scoringLowBasket(0.1, robotPower);
                

            }

            //runMacro("ReadyToPickup");
            //runMacro("ReadyToDrop");
        }
    }

    public void scoringLowBasket(double slideMotor1Power, double[] MoveRobotPower) {
         slideMotor1.setPower(slideMotor1Power);
         MoveRobot(MoveRobotPower,0,false);

         
         // if the robot reaches the b positions for both the arm and the position, stop moving
         // for positions, robot will move for 10s, arm will move after 2s and end 2s before the 10s is finished (2s and 8s)
         //after get both arm and position to point b and doing it's task, then it will loop back to the beginning of the code and rerun the entire code
    }
}