/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test Grabber", group = "ZTest")
//@Disabled
public class TestGrabber extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   200;     // period of each cycle
    static final double MAX_POS     =  0.57;     // Maximum rotational position
    static final double MIN_POS     =  0.37;     // Minimum rotational position

    // Define class members
    Servo   grabber;
    double  position = MIN_POS; // Start at halfway position

    boolean open = true;
    boolean pushed = false;
int countopen = 0;
int countclose = 0;
    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        grabber = hardwareMap.get(Servo.class, "grabber");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            /*
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                grabber.setPosition(MIN_POS);
                position = MIN_POS;
            }
            else if (gamepad1.left_bumper && !gamepad1.right_bumper){
                grabber.setPosition(MAX_POS);
                position = MAX_POS;
            }

             */
            // slew the servo, according to the rampUp (direction) variable.
            if ((gamepad1.right_bumper || gamepad1.left_bumper) && !pushed) {
                pushed = true;
                if (open) {grabber.setPosition(MAX_POS);
                    position = MAX_POS;
                    countclose ++;
                    open = false;}
                else{grabber.setPosition(MIN_POS);
                    position = MIN_POS;
                    countopen++;
                    open = true;}
            }
            else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                pushed = false;
            }

            //position = 0.6;
            // Display the current value
            telemetry.addData("grabber Position", "%5.2f", position);
            telemetry.addData("count open", "%d", countopen );
            telemetry.addData("count close", "%d", countclose );
            telemetry.update();

            // Set the servo to the new position and pause;
         }

     }
}
