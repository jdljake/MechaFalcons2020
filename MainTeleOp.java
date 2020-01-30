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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MainTeleOp", group="Linear Opmode")

public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Toggle tgg = new Toggle();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor middleDrive = null;
//    private DcMotor middleDrive2 = null;
    private DcMotorEx leftArmDrive = null;
    private DcMotorEx rightArmDrive = null;
    private DcMotorEx middleArmDrive = null;
    private Servo servoGrabber= null;
    private Servo leftLatchServo= null;
    private Servo rightLatchServo= null;
    private CRServo grabberExtenderServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        middleDrive = hardwareMap.get(DcMotor.class, "middle_drive");
//        middleDrive2 = hardwareMap.get(DcMotor.class, "middle_drive2");
        leftArmDrive = hardwareMap.get(DcMotorEx.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotorEx.class, "right_arm_drive");
        middleArmDrive = hardwareMap.get(DcMotorEx.class, "middle_arm_drive");
        servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");
        leftLatchServo = hardwareMap.get(Servo.class, "left_latch_servo");
        rightLatchServo = hardwareMap.get(Servo.class, "right_latch_servo");
        grabberExtenderServo = hardwareMap.get(CRServo.class, "grabber_extender");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//        middleDrive2.setDirection(DcMotor.Direction.REVERSE);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
        //using encoders to track motor movement for debugging purposes

        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

        //reset encoders during init
        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftArmDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightArmDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        middleArmDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int latchCounter = 2;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double middlePower;
            double armPower;
            double grabberExtenderServoPower;
            double servoGrabberPosition = servoGrabber.getPosition();


            //////////gamepad 1 ////////////////////////

            double driveLeft = -gamepad1.left_stick_y;
            double driveRight = -gamepad1.right_stick_y;
            double turn = 0.00;

            leftPower    = Range.clip(driveLeft,-1.0, 1.0);
            rightPower   = Range.clip(driveRight, -1.0, 1.0);

            if(gamepad1.left_trigger != 0){
                turn  =  -gamepad1.left_trigger;
            }
            if(gamepad1.right_trigger != 0){
                turn  =  gamepad1.right_trigger;
            }

            middlePower = Range.clip(turn, -1.0, 1.0);

            ////////////////////////////////////////////


            //////////gamepad 2 ////////////////////////
            if(gamepad2.right_trigger != 0){
                if(gamepad2.a){
                    armPower = middleArmDrive.getPower();
                    if (armPower > -1.0){
                        armPower += -0.1;
                    }
                } else {
                    armPower = -0.50;
                }
            } else if(gamepad2.left_trigger != 0){
//                armPower = 0.30;
                armPower = 0.50;
            }
//            else if(leftArmDrive.getCurrentPosition() >= 0 || rightArmDrive.getCurrentPosition() >= 0 || rightArmDrive.getCurrentPosition() >= 0){
//                armPower = -0.1; // stall torque is -0.1, but I set it at 0 temporarily just while testing servos
//                armPower = 0;
//            }
            else {
                armPower = 0.0;
//                armPower = -0.1;
            }

            if(gamepad2.right_bumper){
                servoGrabber.setPosition(1); //open
            }
            if(gamepad2.left_bumper){
                servoGrabber.setPosition(0); //close
            }


            if(tgg.toggle(gamepad2.b)){
                if(latchCounter % 2 == 0){
                    leftLatchServo.setPosition(1);
                    rightLatchServo.setPosition(0);
                    latchCounter += 1;
                } else {
                    leftLatchServo.setPosition(0.5);
                    rightLatchServo.setPosition(0.5);
                    latchCounter += 1;
                }
            }

//            if(gamepad2.left_bumper){
//                grabberExtenderServoPower = 1.0;
//            } else if(gamepad2.right_bumper){
//                grabberExtenderServoPower = -1.0;
//            } else {
//                grabberExtenderServoPower = 0;
//            }

            if(gamepad2.a){
                grabberExtenderServoPower = -1.0;
            } else {
                grabberExtenderServoPower = 0;
            }

            ////////////////////////////////////////////

            //locks the motors in place when not moving while opModeIsActive
//            if (!rightArmDrive.isBusy() && !leftArmDrive.isBusy() && !middleArmDrive.isBusy()){
//                leftArmDrive.setTargetPosition(leftArmDrive.getCurrentPosition());
//                rightArmDrive.setTargetPosition(rightArmDrive.getCurrentPosition());
//                middleArmDrive.setTargetPosition(middleArmDrive.getCurrentPosition()):
//                rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftArmDrive.setVelocity(200);
//                leftArmDrive.setVelocity(200);
//                armPower = 0;
//            }

            // Send calculated power to motors
//            leftDrive.setPower(leftPower);
//            rightDrive.setPower(rightPower);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            middleDrive.setPower(middlePower);
//            middleDrive2.setPower(middlePower);
            leftArmDrive.setPower(armPower);
            rightArmDrive.setPower(armPower);
            middleArmDrive.setPower(armPower);
            grabberExtenderServo.setPower(grabberExtenderServoPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), middle (%.2f), arm (%.2f)", leftPower, rightPower, middlePower, armPower);
            telemetry.addData("leftDrive position", leftDrive.getCurrentPosition());
            telemetry.addData("rightDrive position", rightDrive.getCurrentPosition());
            //telemetry.addData("grabberExtenderServo power:", grabberExtenderServo.getPower());
            //telemetry.addData("latchCounter:", latchCounter);
            telemetry.addData("LeftArmPos", leftArmDrive.getCurrentPosition());
            telemetry.addData("RightArmPos", rightArmDrive.getCurrentPosition());
            telemetry.addData("MiddleArmPos", middleArmDrive.getCurrentPosition());


//            telemetry.addData("latchDown", latchServoDown);
//            telemetry.addData("leftLatchServo position", leftLatchServo.getPosition());
//            telemetry.addData("rightLatchServo position", rightLatchServo.getPosition());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f), middle (%.2f)", leftPower, rightPower, middlePower);
//            telemetry.addData("Servo Position", "%.2f", servoGrabberPosition);
            telemetry.update();
            tgg.reset();
        }
    }
}
