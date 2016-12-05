package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by summitrobotics on 11/16/2016.
 */
@TeleOp(name="Drive :)", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class nicksCode extends LinearOpMode {

    DcMotor lfMotor;
    DcMotor lbMotor;
    DcMotor rfMotor;
    DcMotor rbMotor;

    float lJS;
    float rJS;

    @Override
    public void runOpMode() throws InterruptedException {

        rfMotor = hardwareMap.dcMotor.get("right_front");
        rbMotor = hardwareMap.dcMotor.get("right_back");
        lfMotor = hardwareMap.dcMotor.get("left_front");
        lbMotor = hardwareMap.dcMotor.get("left_back");
        waitForStart();
        while (opModeIsActive()){

            lJS = gamepad1.left_stick_y;
            rJS = gamepad1.right_stick_y;

            lfMotor.setPower(lJS);
            lbMotor.setPower(lJS);
            rfMotor.setPower(rJS);
            rbMotor.setPower(rJS);

        }

    }

}
