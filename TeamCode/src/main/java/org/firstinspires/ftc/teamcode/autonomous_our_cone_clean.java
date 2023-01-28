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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Objects;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: RobotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backward for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Wallace autonomous our cone clean", group = "Wallace")
//@Disabled
public class autonomous_our_cone_clean extends LinearOpMode {

    /* Declare OpMode members. */
    /* First our motors */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotorSimple leftBackDrive = null; // the left back motor is powered by a spark controler which does not have a rotation sensor input - so it is DcMotorSimple
    private DcMotor rightBackDrive = null;
    /* one servo to grab the cone */
    private Servo grabber = null;
    /* The inbuild IMU to find where we are heading */
    IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    // Set COUNTS_PER_INCH for your specific drive train.
    /* We use Rev Motors with a combined x4 and x5 gear ratio (=20) */
    static final double COUNTS_PER_MOTOR_REV = 28;    // Rev Motor Encoder from specs
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // x4 and x5 gear ratios
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // We have 4'' mecanum wheels, likely they are not exactly 4'', the distance driven later is just approximate and we have to finetune for the actual distance driven
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    /* motor power - we have 30 seconds and do not have to hurry. Doing it slow makes the turns easier */
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = -0.2;
    static final double CLOSE_POS = 0.6;     // Closing position of grabber servo
    static final long SLEEP_MS = 10; // ms to sleep between IMU refreshes
    /* This is our model we downloaded from the machine learning website */
    private static final String TFOD_MODEL_ASSET = "model_20230106_083116_cone_version_2.tflite";

    /* This is weird and we need to follow this up. Somehow the order is alphabetically, our circle should be "3 c" but doing this always found the triangle. This labelling here works but it is not what we really want to use. Incidentally the predefined model uses Bolt, Bulb and Panel, also in alphabetical order */
    private static final String[] LABELS = {
            "1 c",
            "2 r",
            "3 t"
    };

    /* We had to make an account with Vuforia and download this key to use this */
    private static final String VUFORIA_KEY =
            "AWg6avH/////AAABmVwGucholUoCjMJvG6Nkzm9T5d2W4ip+kZpZPSLyNRxFFzzirrh9S2aguseh3zkQslKCyjyXTMJDAy4EpbEET+bdgXeAofWJSKMwFfq/qv8wImEVyaS2O15XsqX+uhfqT/jc8dVYvvaM53xe3MmI9yKfcAuneyXZvbxZRjAWTZQjgil1piyQoNA2/bH1ZaxNmEKrHrGOBeFYS27v4erDv7LrukYnTf5zI6oROPNHzFx5mzpUDja+0gi05NFw7Y2d7CyH9fdC1cXj+meHMGHxWVWVzAfOpi6jz9SHQgJsMmG48U7btY10cpBOTkj7PUj7bUxU9enuAT/6IuKR4PywKcIdkiMxVWnW7B0XAfvuuFgO";
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start autonomous Wallace");


        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor1");
        leftBackDrive = hardwareMap.get(DcMotorSimple.class, "motor3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");
        grabber = hardwareMap.get(Servo.class, "grabber");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        /* set up our front motors to use the encoder, in principal we only need one for driving straight since we use the IMU for turning */
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* set up the IMU - our controler is vertical, perpendicular to the driving direction, yaw will give us the angle of our driving direction */
        /* since these are all right angles we do not have to use rotation angles to initialize the IMU */
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        String object_id = "0";
        if (opModeIsActive()) {
            while (Objects.equals(object_id, "0")) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            object_id = recognition.getLabel();
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                        //break;
                    }
                }
            }
        }
        telemetry.addData("found ", "%s", object_id);
        telemetry.update();
        sleep(1000);
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        encoderDrive(DRIVE_SPEED, 0.5, 0.5, 5.0);  // S1: Forward 47
        grabber.setPosition(CLOSE_POS); // close our grabber so we drag our cone along and are able to use it once teleop starts
        sleep(1000);
        if (Objects.equals(object_id, "3 t")) { // drive to loaction 2
            /* This just needs a straight drive (our robot is lined up with the wall to do this) */
            encoderDrive(DRIVE_SPEED, 25, 25, 5.0);  // S1: Forward 47
        } else if (Objects.equals(object_id, "2 r")) { // drive to location 1
            /* Here we need to make a left turn, drive a bit and then a right turn and drive a bit more */
            telemetry.addData("found ", "%s", object_id);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            TurnLeft(TURN_SPEED);


            while (orientation.getYaw(AngleUnit.DEGREES) < 80) {
                sleep(SLEEP_MS); // sleep a bit so we do not bog down the cpu
                telemetry.addData("found ", "%s", object_id);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                orientation = imu.getRobotYawPitchRollAngles();
            }
            MotorsOff();
            encoderDrive(DRIVE_SPEED, 22, 22, 5.0);  // S1: Forward 47
            TurnRight(TURN_SPEED);
            while (orientation.getYaw(AngleUnit.DEGREES) > 10) {
                sleep(SLEEP_MS); // sleep a bit so we do not bog down the cpu
                telemetry.addData("found ", "%s", object_id);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                orientation = imu.getRobotYawPitchRollAngles();
            }
            MotorsOff();
            encoderDrive(DRIVE_SPEED, 30, 30, 5.0);  // S1: Forward 47
        } else if (Objects.equals(object_id, "1 c")) { // drive to location 3
            /* Here we need to make a right turn, drive a bit, make a left turn and drive a bit more */
            telemetry.addData("found ", "%s", object_id);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            TurnRight(TURN_SPEED);
            /* turn until the angle is -80 degrees. With the time lags we are around -90 when the robot stops */
            while (orientation.getYaw(AngleUnit.DEGREES) > -80) {
                sleep(SLEEP_MS); // sleep a bit so we do not bog down the cpu
                telemetry.addData("found ", "%s", object_id);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                orientation = imu.getRobotYawPitchRollAngles();
            }
            MotorsOff();
            encoderDrive(DRIVE_SPEED, 22, 22, 5.0);  // S1: Forward 47
            TurnLeft(TURN_SPEED);
            while (orientation.getYaw(AngleUnit.DEGREES) < -10) {
                sleep(SLEEP_MS); // sleep a bit so we do not bog down the cpu
                telemetry.addData("found ", "%s", object_id);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();
                orientation = imu.getRobotYawPitchRollAngles();
            }
            MotorsOff();
            encoderDrive(DRIVE_SPEED, 30, 30, 5.0);  // S1: Forward 47
            telemetry.addData("we have arrived at ", "%s", object_id);
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }
    }

    /*
     * These are just convenience methods to reduce the number of duplicated lines of code
     * The first one just stops the robot (turns off all motors)
     * The second one starts turning the robot to the left
     * The third one starts a turn to the right
     * The forth one set the motor powers to initiate a left/right turn depending on the first argument
     */
    public void MotorsOff() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    public void TurnLeft(double turnpower) {
        Turn(true, turnpower);
    }

    public void TurnRight(double turnpower) {
        Turn(false, turnpower);
    }

    public void Turn(boolean left, double turnpower) {
        if (left) {
            turnpower = -turnpower;
        }
        double leftFrontPower = turnpower;
        double rightFrontPower = -turnpower;
        double leftBackPower = turnpower;
        double rightBackPower = -turnpower;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        telemetry.addData("lfin", "%.0f %.0f / %.0f", speed, leftInches, rightInches);
        telemetry.update();
        //   sleep(10000);  // pause to display final telemetry message.

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            MotorsOff();

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // We program with Android studio, so this is what we have to use
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}