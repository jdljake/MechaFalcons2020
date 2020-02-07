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

import java.sql.Driver;


@Autonomous(name="BLUEPlanBAuton", group="Pushbot")
public class BLUEPlanBAuton extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx leftArmDrive = null;
    private DcMotorEx middleArmDrive = null;
    private DcMotorEx rightArmDrive = null;

    private Servo grabberExtenderServo = null;
    private Servo servoGrabber= null;
    private Servo leftLatchServo= null;
    private Servo rightLatchServo= null;


    private NormalizedColorSensor colorSensor = null;
    private NormalizedColorSensor tapeColorSensor = null;

    //imu stuff
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    //

    //    static final double     COUNTS_PER_MOTOR    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // should be REV 20:1 HD HEX motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     COUNTS_PER_DEGREE       = 18.8888888889;
    static final double     COUNTS_PER_DEGREE       = 18.8888888889;
    //    static final double     DRIVE_SPEED             = 0.7;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         */
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        leftArmDrive = hardwareMap.get(DcMotorEx.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotorEx.class, "right_arm_drive");
        middleArmDrive = hardwareMap.get(DcMotorEx.class, "middle_arm_drive");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        tapeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "tape_sensor_color");
        servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");
        grabberExtenderServo = hardwareMap.get(Servo.class, "grabber_extender");
        leftLatchServo = hardwareMap.get(Servo.class, "left_latch_servo");
        rightLatchServo = hardwareMap.get(Servo.class, "right_latch_servo");
        //servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        frontRightDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        backLeftDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);
        backRightDrive.setVelocityPIDFCoefficients(5.17, 0.117, 0, 11.7);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
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


        grabberExtenderServo.setPosition(0.9); //continuous, so position is actually power
        encoderLift(-1.0, -200, 2.0);
        sleep(1100);
        grabberExtenderServo.setPosition(0.5); //continuous, so position is actually power
        encoderLiftDrop(2.0);


        encoderDrive(DRIVE_SPEED, 30, 5.0);
        encoderStrafeDrive(DRIVE_SPEED, 24, 5.0);

        servoGrabber.setPosition(1);
        sleep(1000);

        checkForSkystone();

        encoderDrive(DRIVE_SPEED, 14, 5.0);
        servoGrabber.setPosition(0);
        sleep(2000);
        encoderDrive(DRIVE_SPEED, -18, 5.0);

        checkForBridgeTape();

        rotate(-90, 0.3);


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

    public void encoderStrafeDrive(double maxspeed, double inches, double timeoutS){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double currentFrontLeftPower;
        double currentFrontRightPower;
        double currentBackLeftPower;
        double currentBackRightPower;

        inches = 1.11111 * inches;

        if (opModeIsActive()) {
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy()|| frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {

                currentFrontLeftPower = frontLeftDrive.getPower();
                currentFrontRightPower = frontRightDrive.getPower();
                currentBackLeftPower = backLeftDrive.getPower();
                currentBackRightPower = backRightDrive.getPower();

                if(currentFrontLeftPower < maxspeed){
                    frontLeftDrive.setPower(currentFrontLeftPower + 0.01);
                }
                if(currentFrontRightPower < maxspeed){
                    frontRightDrive.setPower(currentFrontRightPower + 0.01);
                }
                if(currentBackLeftPower < maxspeed){
                    backLeftDrive.setPower(currentBackLeftPower + 0.01);
                }
                if(currentBackRightPower < maxspeed){
                    backRightDrive.setPower(currentBackRightPower + 0.01);
                }

                // Display it for the driver.
                telemetry.addData("Strafe Time: ", "its strafe time");
                telemetry.addData("FLTarget", newFrontLeftTarget);
                telemetry.addData("FRTarget", newFrontRightTarget);
                telemetry.addData("BLTarget", newBackLeftTarget);
                telemetry.addData("BRTarget", newBackRightTarget);

                telemetry.addData("FLPos", frontLeftDrive.getCurrentPosition());
                telemetry.addData("FRPos", frontRightDrive.getCurrentPosition());
                telemetry.addData("BLPos", backLeftDrive.getCurrentPosition());
                telemetry.addData("BRPos", backRightDrive.getCurrentPosition());

                telemetry.addData("FLPower", frontLeftDrive.getPower());
                telemetry.addData("FRPower", frontRightDrive.getPower());
                telemetry.addData("BLPower", backLeftDrive.getPower());
                telemetry.addData("BRPower", backRightDrive.getPower());
                telemetry.update();
            }


            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double maxspeed, double inches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double currentFrontLeftPower;
        double currentFrontRightPower;
        double currentBackLeftPower;
        double currentBackRightPower;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy()|| frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {

                currentFrontLeftPower = frontLeftDrive.getPower();
                currentFrontRightPower = frontRightDrive.getPower();
                currentBackLeftPower = backLeftDrive.getPower();
                currentBackRightPower = backRightDrive.getPower();

                if(currentFrontLeftPower < maxspeed){
                    frontLeftDrive.setPower(currentFrontLeftPower + 0.01);
                }
                if(currentFrontRightPower < maxspeed){
                    frontRightDrive.setPower(currentFrontRightPower + 0.01);
                }
                if(currentBackLeftPower < maxspeed){
                    backLeftDrive.setPower(currentBackLeftPower + 0.01);
                }
                if(currentBackRightPower < maxspeed){
                    backRightDrive.setPower(currentBackRightPower + 0.01);
                }

                // Display it for the driver.
                telemetry.addData("FLTarget", newFrontLeftTarget);
                telemetry.addData("FRTarget", newFrontRightTarget);
                telemetry.addData("BLTarget", newBackLeftTarget);
                telemetry.addData("BRTarget", newBackRightTarget);

                telemetry.addData("FLPos", frontLeftDrive.getCurrentPosition());
                telemetry.addData("FRPos", frontRightDrive.getCurrentPosition());
                telemetry.addData("BLPos", backLeftDrive.getCurrentPosition());
                telemetry.addData("BRPos", backRightDrive.getCurrentPosition());

                telemetry.addData("FLPower", frontLeftDrive.getPower());
                telemetry.addData("FRPower", frontRightDrive.getPower());
                telemetry.addData("BLPower", backLeftDrive.getPower());
                telemetry.addData("BRPower", backRightDrive.getPower());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    //end encoder drive stuff

    //encoder lift
    public void encoderLift(double maxspeed, double ticks, double timeoutS) {
        int newLeftArmTarget;
        int newMiddleArmTarget;
        int newRightArmTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftArmTarget = leftArmDrive.getCurrentPosition() + (int) (ticks);
            newMiddleArmTarget = middleArmDrive.getCurrentPosition() + (int) (ticks);
            newRightArmTarget = rightArmDrive.getCurrentPosition() + (int) (ticks);

            leftArmDrive.setTargetPosition(newLeftArmTarget);
            middleArmDrive.setTargetPosition(newMiddleArmTarget);
            rightArmDrive.setTargetPosition(newRightArmTarget);

            // Turn On RUN_TO_POSITION
            leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftArmDrive.isBusy()|| middleArmDrive.isBusy() || rightArmDrive.isBusy())) {

                leftArmDrive.setPower(-1.0);
                middleArmDrive.setPower(-1.0);
                rightArmDrive.setPower(-1.0);

                // Display it for the driver.
                telemetry.addData("LeftArm Target: ", leftArmDrive.getPower());
                telemetry.addData("MiddleArm Target: ", middleArmDrive.getPower());
                telemetry.addData("RightArm Target: ", rightArmDrive.getPower());

                telemetry.addData("LeftArm Pos", frontLeftDrive.getCurrentPosition());
                telemetry.addData("MiddleArm Pos", frontRightDrive.getCurrentPosition());
                telemetry.addData("RightArm Pos", backLeftDrive.getCurrentPosition());

                telemetry.addData("LeftArm Power", leftArmDrive.getPower());
                telemetry.addData("MiddleArm Power", middleArmDrive.getPower());
                telemetry.addData("RightArm Power", rightArmDrive.getPower());
                telemetry.update();
            }

            if(leftArmDrive.getCurrentPosition() <= 22 || rightArmDrive.getCurrentPosition() <= 22 || middleArmDrive.getCurrentPosition() <= 22){
                // Apply stall torque;
                leftArmDrive.setPower(-0.1);
                middleArmDrive.setPower(-0.1);
                rightArmDrive.setPower(-0.1);
            } else {
                // Stop all motion;
                leftArmDrive.setPower(0);
                middleArmDrive.setPower(0);
                rightArmDrive.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderLiftDrop(double timeoutS) {
        int newLeftArmTarget;
        int newMiddleArmTarget;
        int newRightArmTarget;

        double currentLeftArmPower;
        double currentMiddleArmPower;
        double currentRightArmPower;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftArmTarget = 20;
            newMiddleArmTarget = 20;
            newRightArmTarget = 20;

            leftArmDrive.setTargetPosition(newLeftArmTarget);
            middleArmDrive.setTargetPosition(newMiddleArmTarget);
            rightArmDrive.setTargetPosition(newRightArmTarget);

            // Turn On RUN_TO_POSITION
            leftArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftArmDrive.isBusy()|| middleArmDrive.isBusy() || rightArmDrive.isBusy())) {

                currentLeftArmPower = leftArmDrive.getPower();
                currentMiddleArmPower = middleArmDrive.getPower();
                currentRightArmPower = rightArmDrive.getPower();

                if(currentLeftArmPower < 0.4){
                    leftArmDrive.setPower(currentLeftArmPower + 0.01);
                }
                if(currentMiddleArmPower < 0.4){
                    middleArmDrive.setPower(currentMiddleArmPower + 0.01);
                }
                if(currentRightArmPower < 0.4){
                    rightArmDrive.setPower(currentRightArmPower + 0.01);
                }

                // Display it for the driver.
                telemetry.addData("LeftArm Target: ", currentLeftArmPower);
                telemetry.addData("MiddleArm Target: ", currentMiddleArmPower);
                telemetry.addData("RightArm Target: ", currentRightArmPower);

                telemetry.addData("LeftArm Pos", frontLeftDrive.getCurrentPosition());
                telemetry.addData("MiddleArm Pos", frontRightDrive.getCurrentPosition());
                telemetry.addData("RightArm Pos", backLeftDrive.getCurrentPosition());

                telemetry.addData("LeftArm Power", leftArmDrive.getPower());
                telemetry.addData("MiddleArm Power", middleArmDrive.getPower());
                telemetry.addData("RightArm Power", rightArmDrive.getPower());
                telemetry.update();
            }

            // Turn off RUN_TO_POSITION
            leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //end encoder lift




    ///gyro stuff

    /**
     * Resets the cumulative angle tracking to zero.
     */
    /*private void dropLatches(){
        leftLatchServo.setPosition(1);
        rightLatchServo.setPosition(0);
        sleep(2000);
    }
    private void raiseLatches(){
        rightLatchServo.setPosition(0.5);
        leftLatchServo.setPosition(0.5);
        sleep(2000);
    }*/
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
        double  frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            frontLeftPower = power;
            backLeftPower = power;
            frontRightPower = -power;
            backRightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            frontLeftPower = -power;
            backLeftPower = -power;
            frontRightPower = power;
            backRightPower = power;

        }
        else return;

        // set power to rotate.
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);

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
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

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
        while (opModeIsActive() && !skystoneseen && (runtime.seconds() < 5.0)) {
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
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                encoderStrafeDrive(0.4, -6, 7.0);
                break;
            } else {
                skystoneseen = false;
                frontLeftDrive.setPower(-0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(-0.5);
            }

//            telemetry.addData("Color Condition", colorCondition);
            telemetry.addData("Skystone:", skystoneseen);
            telemetry.update();

        }

        while(opModeIsActive() && skystoneseen){
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
                frontLeftDrive.setPower(-0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(-0.5);
            } else {
                skystoneseen = false;
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                encoderStrafeDrive(0.4, -6.4, 7.0);
                break;
            }

//            telemetry.addData("Color Condition", colorCondition);
            telemetry.addData("Skystone:", skystoneseen);
            telemetry.update();
        }

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void checkForBridgeTape(){

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean tapeseen = false;

        // Loop until we are asked to stop
        while (opModeIsActive() && !tapeseen) {
            // Read the sensor
            NormalizedRGBA colors = tapeColorSensor.getNormalizedColors();

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

            if(colorCondition < 1){
                tapeseen = true;
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
//                rotate(84, 0.4);
                encoderStrafeDrive(1.0, -30, 5.0);
                break;
            } else {
                tapeseen = false;
                frontLeftDrive.setPower(-0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(-0.5);
            }

//            telemetry.addData("Color Condition", colorCondition);
            telemetry.addData("Tape:", tapeseen);
            telemetry.update();

        }

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ///end color stuff

}
