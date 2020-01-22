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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoPlay", group="Pushbot")
public class AutoPlay extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx middleDrive = null;
    private DcMotorEx middleDrive2 = null;
    private Servo leftLatchServo = null;
    private Servo rightLatchServo = null;
    //private Servo servoGrabber= null;
    private NormalizedColorSensor colorSensor = null;

    //imu stuff
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    //

    //    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // should be REV 20:1 HD HEX motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_DEGREE       = 18.8888888889;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         */
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        middleDrive = hardwareMap.get(DcMotorEx.class, "middle_drive");
        middleDrive2 = hardwareMap.get(DcMotorEx.class, "middle_drive2");
        leftLatchServo = hardwareMap.get(Servo.class, "left_latch_servo");
        rightLatchServo = hardwareMap.get(Servo.class, "right_latch_servo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        //servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive2.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        middleDrive2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        rightDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        middleDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        middleDrive2.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Counts Per Inch:",  "%.2f", COUNTS_PER_INCH);
        //telemetry.addData("leftLatchServo position", leftLatchServo.getPosition());
        //telemetry.addData("rightLatchServo position", rightLatchServo.getPosition());
        telemetry.update();

        //////////////////////////// imu stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        ////////////////////////////

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


// Step through each leg of the path,
// Note: Reverse movement is obtained by setting a negative distance (not speed)
////////////////////////////////////////////////////////////////////////////////////



        //Moving Base to construction zone
//        encoderDrive(DRIVE_SPEED,  30,  30, 5.0);
//        rotate(-90, 0.7);
//        leftLatchServo.setPosition(1);
//        rightLatchServo.setPosition(0);
//        encoderMiddleDrive(DRIVE_SPEED,  -22,  5.0);
//        leftLatchServo.setPosition(0.5);
//        rightLatchServo.setPosition(0.5);
//        rotate(90, 0.7);


        encoderDrive(DRIVE_SPEED, 16, 16, 5.0);
        checkForSkystone(); //runs until skystone seen, so assume that skystone is found in next step
        encoderMiddleDrive(0.4, -8, 5.0);
        encoderDrive(DRIVE_SPEED, 3, 3, 5.0);

//        encoderMiddleDrive(DRIVE_SPEED,  62,  5.0);

//        encoderDrive(DRIVE_SPEED,  34,  34, 5.0);
        //servoGrabber.setPosition(0.9);
//        encoderDrive(DRIVE_SPEED,  -34,  -34, 5.0);
//        rotate(90, 0.7);
//        encoderDrive(DRIVE_SPEED,  60,60  , 5.0);
        //servoGrabber.setPosition(0.32);
//        rotate(-90, 0.7);

        //Dock
//        encoderMiddleDrive(DRIVE_SPEED,  -22,  5.0);



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    ///encoder drive stuff

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double currentLeftPower;
        double currentRightPower;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
//            leftDrive.setPower(Math.abs(speed));
//            rightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy())) {

                currentLeftPower = leftDrive.getPower();
                currentRightPower = rightDrive.getPower();

                if(currentLeftPower < 1.0){
                    leftDrive.setPower(currentLeftPower + 0.01);
                }
                if(currentRightPower < 1.0){
                    rightDrive.setPower(currentRightPower + 0.01);
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.addData("currentLeftPower", leftDrive.getPower());
                telemetry.addData("currentRightPower", rightDrive.getPower());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    private void encoderDriveAngle(double speed, double angle, double timeoutS) {
        int newLeftAngleTarget;
        int newRightAngleTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftAngleTarget = leftDrive.getCurrentPosition() + (int) (angle * COUNTS_PER_DEGREE);
            newRightAngleTarget = rightDrive.getCurrentPosition() + (int) (-angle * COUNTS_PER_DEGREE);
            leftDrive.setTargetPosition(newLeftAngleTarget);
            rightDrive.setTargetPosition(newRightAngleTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftAngleTarget, newRightAngleTarget);
                telemetry.addData("LeftDrivePos", leftDrive.getCurrentPosition());
                telemetry.addData("RightDrivePos", rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }

    }

    private void encoderMiddleDrive(double speed, double middleInches, double timeoutS) {

        int newMiddleTarget;
        double currentMiddlePower;
        double currentMiddle2Power;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = middleDrive.getCurrentPosition() + (int)(middleInches * COUNTS_PER_INCH);
            middleDrive.setTargetPosition(newMiddleTarget);
            middleDrive2.setTargetPosition(newMiddleTarget);

            // Turn On RUN_TO_POSITION
            middleDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
//            middleDrive.setPower(Math.abs(speed));
//            middleDrive2.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && middleDrive.isBusy()) {

                currentMiddlePower = middleDrive.getPower();
                currentMiddle2Power = middleDrive2.getPower();

                if(currentMiddlePower < 1.0){
                    middleDrive.setPower(currentMiddlePower + 0.01);
                }
                if(currentMiddle2Power < 1.0){
                    middleDrive2.setPower(currentMiddle2Power + 0.01);
                }

                // Display it for the driver.
                telemetry.addData("Path1, Running to:", newMiddleTarget);
                telemetry.addData("MiddleDrivePos", middleDrive.getCurrentPosition());
                telemetry.addData("MiddleDrive2Pos", middleDrive2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            middleDrive.setPower(0);
            middleDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //end encoder drive stuff


    ///gyro stuff

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;

        }
        else return;

        // set power to rotate.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("getAngleDuringRightTurn()", getAngle());
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("getAngleDuringLeftTurn()", getAngle());
                telemetry.update();
            }

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    ///end gyro stuff



    ///start color stuff

    protected void checkForSkystone(){

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean skystoneseen = false;

        // Loop until we are asked to stop
        while (opModeIsActive() && !skystoneseen) {
            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            int color = colors.toColor();

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            float colorCondition;
            float r = Color.red(color);
            float g = Color.green(color);
            float b = Color.blue(color);
            colorCondition = (r * g) / (b * b);

            if(colorCondition < 2){
                skystoneseen = true;
                middleDrive.setPower(0);
                middleDrive2.setPower(0);
            } else {
                skystoneseen = false;
                middleDrive.setPower(-0.3);
                middleDrive2.setPower(0.3);
            }

//            telemetry.addData("Color Condition", colorCondition);
            telemetry.addData("Skystone:", skystoneseen);
            telemetry.update();

        }
    }

    ///end color stuff

}
