package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FourWheelDrive", group="Linear Opmode")

public class FourWheelDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Toggle tgg = new Toggle();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor middleDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
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
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
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
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
        //using encoders to track motor movement for debugging purposes

        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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



            if(gamepad1.right_bumper){
                leftPower    = Range.clip(driveLeft,-1.0, 1.0);
                rightPower   = Range.clip(driveRight, -1.0, 1.0);
            } else if (gamepad1.left_bumper){
                leftPower    = Range.clip(driveLeft,-0.25, 0.25);
                rightPower   = Range.clip(driveRight, -0.25, 0.25);
            }
            else {
                leftPower    = Range.clip(driveLeft,-0.5, 0.5);
                rightPower   = Range.clip(driveRight, -0.5, 0.5);
            }


            if(gamepad1.left_trigger != 0){
                turn  =  -gamepad1.left_trigger;
            }
            if(gamepad1.right_trigger != 0){
                turn  =  gamepad1.right_trigger;
            }

            middlePower = Range.clip(turn, -1.0, 1.0);

            if(gamepad1.b){

            }

            ////////////////////////////////////////////


            //////////gamepad 2 ////////////////////////

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
                armPower = -0.1; // stall torque is -0.1, but I set it at 0 temporarily just while testing servos
            }
            else {
                armPower = 0.0;
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

            if(gamepad2.a){
                grabberExtenderServoPower = -1.0;
            } else {
                grabberExtenderServoPower = 0;
            }


            if(gamepad2.dpad_left){
                leftPower -= 0.1;
                rightPower -= 0.1;
            }
            else if(gamepad2.dpad_right){
                leftPower += 0.1;
                rightPower += 0.1;
            }


            ////////////////////////////////////////////


            // Send calculated power to motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            middleDrive.setPower(middlePower);
            leftArmDrive.setPower(armPower);
            rightArmDrive.setPower(armPower);
            middleArmDrive.setPower(armPower);
            grabberExtenderServo.setPower(grabberExtenderServoPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), middle (%.2f)", leftPower, rightPower, middlePower);
            telemetry.addData("Left Back Wheel Actual Power: ", "leftBack (%.2f)", leftBackDrive.getPower());
            telemetry.addData("Right Back Wheel Actual Power: ", "rightBack (%.2f)", rightBackDrive.getPower());
            telemetry.addData("leftArm Power", leftArmDrive.getPower());
            telemetry.addData("middleArm Power", middleDrive.getPower());
            telemetry.addData("rightArm Power", rightArmDrive.getPower());

            telemetry.addData("leftDrive position", leftDrive.getCurrentPosition());
            telemetry.addData("rightDrive position", rightDrive.getCurrentPosition());
            telemetry.addData("leftArm position", leftArmDrive.getCurrentPosition());
            telemetry.addData("middleArm position", middleArmDrive.getCurrentPosition());
            telemetry.addData("rightArm position", rightArmDrive.getCurrentPosition());

            telemetry.update();
            tgg.reset();
        }
    }
}
