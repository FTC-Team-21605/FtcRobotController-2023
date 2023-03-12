/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Wallace TeleOp w elevator limit", group = "Wallace")
//@Disabled
public class TeleopWithElevatorLimit extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotorSimple leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor elevator = null;
    private Servo grabber = null;
    static final double CLOSE_POS = 0.57;     // Maximum rotational position
    static final double OPEN_POS = 0.37;     // Minimum rotational position
    int countopen = 0;
    int countclose = 0;
    // elevator variables
    //int elevatorposition_start = 0;
    static final int LOW_POLE = 870;
    static final int MEDIUM_POLE = 1300;
    static final int HIGH_POLE = 1830;
    static final double LOW_POLE_SPEED = 0.7;
    static final double MEDIUM_POLE_SPEED = 0.7;
    static final double HIGH_POLE_SPEED = 0.8;
    static final double DOWN_SPEED = -0.5;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive = hardwareMap.get(DcMotorSimple.class, "motor3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");

        grabber = hardwareMap.get(Servo.class, "grabber");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

// elevator motor setup
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set to break on zero power to hold position
        // hokey way to reset encoder
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elevatorposition_start = elevator.getCurrentPosition();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int elevatorposition_start = elevator.getCurrentPosition();
        int elevator_moveto = elevatorposition_start;
        double elevator_fixed_speed = 0;
        double elevatorPower = 0;
        boolean move_up = true;
        boolean dpad_pressed = false;
        int move_down_offset = 0;

        boolean open = false;
        boolean pushed = false;
        double grabber_position = CLOSE_POS; // Start at halfway position
        grabber.setPosition(grabber_position);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y / 2.;  // Note: pushing stick forward gives negative value
            double lateral = 0.;
            if (Math.abs(gamepad1.left_stick_x)>0.8) {
                lateral = gamepad1.left_stick_x / 2.;
            }
            double yaw = gamepad1.right_stick_x / 2.;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

// elevator
            int current_elevator_position = elevator.getCurrentPosition() - elevatorposition_start;
            if (gamepad1.a) {
                elevator_moveto = LOW_POLE;
                if (current_elevator_position < LOW_POLE) {
                    elevatorPower = LOW_POLE_SPEED;
                    move_up = true;
                } else if (current_elevator_position > LOW_POLE) {
                    move_up = false;
                    elevatorPower = DOWN_SPEED; //-LOW_POLE_SPEED;
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
                    elevatorPower = DOWN_SPEED; //-MEDIUM_POLE_SPEED;
                    move_down_offset = 120;
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
                    elevatorPower = DOWN_SPEED; //-HIGH_POLE_SPEED;
                    move_down_offset = 0;
                } else {
                    elevatorPower = 0;
                }
            }
            if (gamepad1.x) {
                elevator_moveto = Math.max(elevatorposition_start,0);
                if (current_elevator_position < elevatorposition_start) {
                    elevatorPower = 0.5;
                    move_up = true;
                } else if (current_elevator_position > elevatorposition_start) {
                    move_up = false;
                    elevatorPower = DOWN_SPEED;
                    move_down_offset = 0;
                } else {
                    elevatorPower = 0;
                }
            }

            if (gamepad1.dpad_up && !dpad_pressed){
                elevator_moveto = current_elevator_position + 400;
                move_up = true;
                elevatorPower = 0.5;
                dpad_pressed = true;
            }
            else if ( !gamepad1.dpad_up && dpad_pressed) {
                dpad_pressed = false;
            }

            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                elevatorPower = Math.min(gamepad1.right_trigger, 0.8);
                elevator_moveto = -1000;

            } else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                elevatorPower = Math.max(-0.5, -gamepad1.left_trigger);
                elevator_moveto = -1000;
            } else {
                if (elevator_moveto < 0) {
                    elevatorPower = 0;
                }
            }
            if (elevator_moveto >= elevatorposition_start) {
                if (move_up) {
                    if (current_elevator_position >= elevator_moveto) {
                        elevatorPower = 0;
                        elevator_moveto = -1000;
                    }
                } else {
                    if (current_elevator_position <= (elevator_moveto + move_down_offset)) {
                        elevatorPower = 0;
                        elevator_moveto = -1000;
                    }
                }
            }
            if (elevatorPower < 0 && current_elevator_position <= 0) {
                elevatorPower = 0;
            }

            elevator.setPower(elevatorPower);
            telemetry.addData("elevator position", "%d", current_elevator_position);
            if ((gamepad1.right_bumper || gamepad1.left_bumper) && !pushed) {
                pushed = true;
                if (open) {
                    grabber.setPosition(CLOSE_POS);
                    grabber_position = CLOSE_POS;
                    open = false;
                } else {
                    grabber.setPosition(OPEN_POS);
                    grabber_position = OPEN_POS;
                    open = true;
                }
            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                pushed = false;
            }
            //grabber.setPosition(grabber_position);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("axial (ly), lateral (lx), yaw (ry) ", "%4.2f, %4.2f. %4.2f", axial, lateral, yaw);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Grabber Position", "%4.2f", grabber_position);
            telemetry.addData("Move to, up", "%d, %b",elevator_moveto, move_up);
            telemetry.update();
        }
    }
}
