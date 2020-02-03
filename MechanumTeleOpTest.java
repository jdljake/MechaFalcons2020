package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MechanumTeleOpTest", group="Linear Opmode")

public class MechanumTeleOpTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Toggle tgg = new Toggle();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx leftArmDrive = null;
    private DcMotorEx rightArmDrive = null;
    private DcMotorEx middleArmDrive = null;
    private Servo leftLatchServo= null;
    private Servo rightLatchServo= null;
    private Servo servoGrabber= null;
    private Servo grabberExtenderServo = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        leftArmDrive = hardwareMap.get(DcMotorEx.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotorEx.class, "right_arm_drive");
        middleArmDrive = hardwareMap.get(DcMotorEx.class, "middle_arm_drive");
        servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");
        servoGrabber = hardwareMap.get(Servo.class, "grabber_servo");
        leftLatchServo = hardwareMap.get(Servo.class, "left_latch_servo");
        rightLatchServo = hardwareMap.get(Servo.class, "right_latch_servo");
        grabberExtenderServo = hardwareMap.get(Servo.class, "grabber_extender");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
        //using encoders to track motor movement for debugging purposes

        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //reset encoders during init
        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int latchCounter = 2;
        leftLatchServo.setPosition(0.5);
        rightLatchServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////////gamepad 1 ////////////////////////

            double driveLeft = -gamepad1.left_stick_y;
            double driveRight = -gamepad1.right_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = (gamepad1.left_trigger > gamepad1.right_trigger)? -gamepad1.left_trigger: gamepad1.right_trigger;
            // region Petr: I made this and thought that sex jokes were appropriate
//            double pullOut = Range.clip(gamepad2.left_bumper, 0.5, 0.3); // idk why the min works like that but it does
//            double shoveIn = Range.clip(gamepad2.right_trigger, 0.5, 0.7);

            //endregion
            double servoGrabberPosition = servoGrabber.getPosition();
            double grabberExtenderServoPower;
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;
            double armPower;


            //////////gamepad 1 //////////////////////////////////////////

            if(gamepad1.right_bumper){
                frontLeftPower    = Range.clip(driveLeft + strafe,-1.0, 1.0);
                frontRightPower   = Range.clip(driveRight - strafe, -1.0, 1.0);
                backLeftPower    = Range.clip(driveLeft - strafe,-1.0, 1.0);
                backRightPower   = Range.clip(driveRight + strafe, -1.0, 1.0);
            } else if (gamepad1.left_bumper){
                frontLeftPower    = Range.clip(driveLeft + strafe,-0.25, 0.25);
                frontRightPower   = Range.clip(driveRight - strafe, -0.25, 0.25);
                backLeftPower    = Range.clip(driveLeft - strafe,-0.25, 0.25);
                backRightPower   = Range.clip(driveRight + strafe, -0.25, 0.25);
            }
            else {
                frontLeftPower    = Range.clip(driveLeft + strafe,-0.5, 0.5);
                frontRightPower   = Range.clip(driveRight - strafe, -0.5, 0.5);
                backLeftPower    = Range.clip(driveLeft - strafe,-0.5, 0.5);
                backRightPower   = Range.clip(driveRight + strafe, -0.5, 0.5);
            }

            //////////////////////////////////////////////////////////////


            //////////gamepad 2 //////////////////////////////////////////

            if(gamepad2.right_trigger != 0){
                if(gamepad2.a){
                    if(leftArmDrive.getCurrentPosition() > -380 || middleArmDrive.getCurrentPosition() > -380 || rightArmDrive.getCurrentPosition() > -380) {
                        armPower = middleArmDrive.getPower();
                        if (armPower > -1.0) {
                            armPower += -0.1;
                        }
                    } else {
                        armPower = -0.50;
                    }
                } else {
                    armPower = -0.50;
                }
            } else if(gamepad2.left_trigger != 0){
//                armPower = 0.30;
                armPower = 0.50;
            }
            else if((leftArmDrive.getCurrentPosition() >= -330 && leftArmDrive.getCurrentPosition() <= -10) || (rightArmDrive.getCurrentPosition() >= -330 && rightArmDrive.getCurrentPosition() <= -10) ||
                    (middleArmDrive.getCurrentPosition() >= -330 && middleArmDrive.getCurrentPosition() <= -10)){
                armPower = -0.01; // stall torque is -0.01
            }
            else {
                armPower = 0.0;
            }


            //grabber extender
            if(gamepad2.right_bumper){
                grabberExtenderServoPower = 0.7;
            }else if(gamepad2.left_bumper){
                grabberExtenderServoPower = 0.3;
            }else{
                grabberExtenderServoPower = 0.5;
            }

            // Finite movement
            if(gamepad2.dpad_down){
                frontLeftPower -= 0.15;
                frontRightPower -= 0.15;
                backLeftPower -= 0.15;
                backRightPower -= 0.15;
            }
            else if(gamepad2.dpad_up){
                frontLeftPower += 0.15;
                frontRightPower += 0.15;
                backLeftPower += 0.15;
                backRightPower += 0.15;
            }
            else if(gamepad2.dpad_left){
                frontLeftPower -= 0.2;
                frontRightPower += 0.2;
                backLeftPower += 0.2;
                backRightPower-= 0.2;
            }
            else if(gamepad2.dpad_right){
                frontLeftPower += 0.2;
                frontRightPower -= 0.2;
                backLeftPower -= 0.2;
                backRightPower+= 0.2;
            }


            //latches
            if(tgg.toggle(gamepad1.b)){
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

            //grabber
            if(tgg.toggle(gamepad2.x)){
                if(servoGrabberPosition == 1){
                    servoGrabber.setPosition(0);
                }else{
                    servoGrabber.setPosition(1);
                }
            }

            //////////////////////////////////////////////////////////////


            // Send calculated power to motors
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            leftArmDrive.setPower(armPower);
            rightArmDrive.setPower(armPower);
            middleArmDrive.setPower(armPower);
            grabberExtenderServo.setPosition(grabberExtenderServoPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("FrontLeft Actual Power: ", "leftBack (%.2f)", frontLeftDrive.getPower());
            telemetry.addData("FrontRight Actual Power: ", "rightBack (%.2f)", frontRightDrive.getPower());
            telemetry.addData("BackLeft Actual Power: ", "leftBack (%.2f)", backLeftDrive.getPower());
            telemetry.addData("BackRight Actual Power: ", "rightBack (%.2f)", backRightDrive.getPower());
            telemetry.addData("FrontLeftDrive position: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FrontRightDrive position: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("BackLeftDrive position: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("BackRightDrive position: ", backRightDrive.getCurrentPosition());

            telemetry.update();
            tgg.reset();
        }
    }
}
