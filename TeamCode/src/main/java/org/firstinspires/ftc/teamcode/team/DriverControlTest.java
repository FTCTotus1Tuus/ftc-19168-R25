package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriverControlTest extends LinearOpMode {

    DarienDriveSub drive;

    @Override
    public void runOpMode() throws InterruptedException {

        //INITIALIZATION
        drive = new DarienDriveSub(hardwareMap, telemetry, new String[]{"omniMotor0", "omniMotor1", "omniMotor2", "omniMotor3"}, new DcMotorSimple.Direction[]{DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE});

        drive.runDriveSystem(gamepad1);

        telemetry.update();
    }
}
