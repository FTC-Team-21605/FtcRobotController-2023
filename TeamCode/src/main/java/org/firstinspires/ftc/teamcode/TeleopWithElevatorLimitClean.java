

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Wallace TeleOp w elevator limit clean", group = "Wallace")
@Disabled
public class TeleopWithElevatorLimitClean extends LinearOpMode {

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
    static final int LOW_POLE = 470;
    static final int MEDIUM_POLE = 720;
    static final int HIGH_POLE = 930;
    static final double LOW_POLE_SPEED = 1.;
    static final double MEDIUM_POLE_SPEED = 1.;
    static final double HIGH_POLE_SPEED = 1.;
    static final double DOWN_SPEED = -1.;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive = hardwareMap.get(DcMotorSimple.class, "motor3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        grabber = hardwareMap.get(Servo.class, "grabber");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int elevatorposition_start = elevator.getCurrentPosition();
        int elevator_moveto = elevatorposition_start;
        double elevator_fixed_speed = 0;
        double elevatorPower = 0;
        boolean move_up = true;
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
            double lateral = gamepad1.left_stick_x / 2.;
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
                    elevatorPower = -HIGH_POLE_SPEED;
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

            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                elevatorPower = gamepad1.right_trigger;
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
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Grabber Position", "%4.2f", grabber_position);
            telemetry.addData("Move to, up", "%d, %b",elevator_moveto, move_up);
            telemetry.update();
        }
    }
}
