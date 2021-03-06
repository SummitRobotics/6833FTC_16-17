package org.firstinspires.ftc.teamcode;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="6833 Auto with 7 second delay", group="Autonomous")  // @Autonomous(...) is the other common choice

public class mAuto7 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //Drive Motors
    DcMotor rfDrive;
    DcMotor rbDrive;
    DcMotor lfDrive;
    DcMotor lbDrive;

    final int wheelRPM = 135;
    final double wheelDiameter = 4;
    final float spinTime = (float) 2.78; //Please forgive me
    final float timePerDegree = spinTime / 360;

    float tileLength = 13.5f;

    public void setAllPower(double power){
        rfDrive.setPower(-power);
        rbDrive.setPower(power);
        lfDrive.setPower(power);
        lbDrive.setPower(-power);

    }

    public void driveForTime(double power, double time, Boolean stopAfter){
        try {
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < time) {
                setAllPower(power);

            }
            if(stopAfter){
                setAllPower(0);
            }

        }finally {
            telemetry.addData("Exception Caught", "Something went wrong while trying to driveForTime()");
            telemetry.update();
        }
    }

    public void driveForDistance(float inches, double rawPower, boolean stopAfter){


        double power = Math.abs(rawPower);

        double inPerSec= 1 / ((wheelRPM * power) * (wheelDiameter * PI) / 60);

        driveForTime(rawPower, inPerSec * inches, stopAfter);

    }

    //Negative power for left turn, positive power for right
    public void turnForDegrees(float degrees, float power, boolean stopAfter){
        String Dir;
        if(power > 0){
            Dir = "R";
        }else{
            Dir = "L";
        }
        turnForTime(power, degrees * timePerDegree, Dir, stopAfter);
    }

    //Direction takes only two values! L for left and R for right. Error handling is setup if you do it wrong, but it's still not good

    public void turnForTime(float power,  float time, String direction, Boolean stopAfter){

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time){
            if(direction == "R"){
                rfDrive.setPower(power);
                rbDrive.setPower(power);

                lfDrive.setPower(power);
                lbDrive.setPower(power);
            }
            else if(direction == "L"){
                rfDrive.setPower(power);
                rbDrive.setPower(power);

                lfDrive.setPower(power);
                lbDrive.setPower(power);
            }
            else{
                telemetry.addData("Exception Caught", "turnForTime did not receive a valid direction, received " + direction);
                telemetry.update();
            }



        }
        if(stopAfter){
            setAllPower(0);
        }

    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rfDrive = hardwareMap.dcMotor.get("right_front");
        rbDrive = hardwareMap.dcMotor.get("right_back");
        lfDrive = hardwareMap.dcMotor.get("left_front");
        lbDrive = hardwareMap.dcMotor.get("left_back");
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while(runtime.seconds() < 7 && opModeIsActive()){
            setAllPower(0);
        }

        //Main auto. For power setting please use the abstraction functions above, try not to set the power manually for neatness sake.

        /* start farthest from beacons. hit ball and possible beacon (start 45 to wall facing balls)*/
        driveForDistance(tileLength*(3.5f*1.44f),1,true);

        driveForDistance(tileLength*1,-1,true);
        setAllPower(0);

        /*45 to wall closest to beacons. knock ball and take beacon*/
          /*  driveForDistance(tileLength* 2.5f, 1, true);
            turnForDegrees(90, 1, true);
            driveForDistance(tileLength* 2f, 1, true);
            turnForDegrees(45,1,true);
            driveForDistance(tileLength*3f,1,true); */





    }

}
