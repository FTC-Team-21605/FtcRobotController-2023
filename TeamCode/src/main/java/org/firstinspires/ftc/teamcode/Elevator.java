/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This OpMode executes a basic Tank Drive Teleop for a two wheeled robot using two REV SPARKminis.
 * To use this example, connect two REV SPARKminis into servo ports on the Expansion Hub. On the
 * robot configuration, use the drop down list under 'Servos' to select 'REV SPARKmini Controller'
 * and name them 'left_drive' and 'right_drive'.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Test: Elevator", group = "ZTest")
//@Disabled
public class Elevator extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor elevator = null;
    //private DcMotorSimple rightDrive = null;
    static final int LOW_POLE = 330;
    static final int MEDIUM_POLE = 500;
    static final int HIGH_POLE = 930;
    static final double LOW_POLE_SPEED = 0.5;
    static final double MEDIUM_POLE_SPEED = 0.8;
    static final double HIGH_POLE_SPEED = 1.;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        elevator.setDirection(DcMotor.Direction.FORWARD);
        int elevatorposition_start = elevator.getCurrentPosition();
        int elevator_moveto = elevatorposition_start;
        double elevator_fixed_speed = 0;
        double elevatorPower = 0;
        boolean move_up = true;
        int move_down_offset = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            int current_elevator_position = elevator.getCurrentPosition() - elevatorposition_start;
             if (gamepad1.a) {
                elevator_moveto = LOW_POLE;
                if (current_elevator_position < LOW_POLE) {
                    elevatorPower = LOW_POLE_SPEED;
                    move_up = true;
                } else if (current_elevator_position > LOW_POLE) {
                    move_up = false;
                    elevatorPower = -LOW_POLE_SPEED;
                    move_down_offset = 50;
                } else {
                    elevatorPower = 0;
                }
            }
            if (gamepad1.b) {
                elevator_moveto = MEDIUM_POLE;
                if (current_elevator_position < MEDIUM_POLE) {
                    elevatorPower = MEDIUM_POLE_SPEED;
                    move_up = true;
                } else if (current_elevator_position > MEDIUM_POLE) {
                    move_up = false;
                    elevatorPower = -MEDIUM_POLE_SPEED;
                    move_down_offset = 125;
                } else {
                    elevatorPower = 0;
                }
            }
            if (gamepad1.y) {
                elevator_moveto = HIGH_POLE;
                if (current_elevator_position < HIGH_POLE) {
                    elevatorPower = HIGH_POLE_SPEED;
                    move_up = true;
                } else if (current_elevator_position > HIGH_POLE) {
                    move_up = false;
                    elevatorPower = -HIGH_POLE_SPEED;
                    move_down_offset = 0;
                } else {
                    elevatorPower = 0;
                }
            }

            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                elevatorPower = gamepad1.right_trigger;
                elevator_moveto = -1000;

            } else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                elevatorPower = -0.5;
                elevator_moveto = -1000;
            } else {
                if (elevator_moveto < 0) {
                    elevatorPower = 0;
                }
            }
            if (elevator_moveto > 0) {
                if (move_up) {
                    if (current_elevator_position >= elevator_moveto) {
                        elevatorPower = 0;
                        elevator_moveto = -1000;
                    }
                } else {
                    if (current_elevator_position <= elevator_moveto) {
                        elevatorPower = 0;
                        elevator_moveto = -1000;
                    }
                }
            }
            if (elevatorPower < 0 && current_elevator_position <= 0) {
                elevatorPower = 0;
            }

            elevator.setPower(elevatorPower);//rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "elevator (%.2f)", elevatorPower);
            telemetry.addData("Motors", "position (%d)", elevator.getCurrentPosition());
            telemetry.update();
        }
    }
}
