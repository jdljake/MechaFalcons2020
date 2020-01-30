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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        backLeftDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

//      //mechanum wheels should be configured as below to match this code:
//
//                                      /  \
//
//                                      \  /
//
//      ////////////////////////////////////////////////////////////////////////


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
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////////gamepad 1 ////////////////////////

            double driveLeft = -gamepad1.left_stick_y;
            double driveRight = -gamepad1.right_stick_y;
//            double turn = gamepad1.right_stick_x;
            double strafe = (gamepad1.left_trigger > gamepad1.right_trigger)? -gamepad1.left_trigger: gamepad1.right_trigger;

            double frontLeftPower = Range.clip(driveLeft + strafe, -1.0, 1.0);
            double backLeftPower = Range.clip(driveLeft - strafe, -1.0, 1.0);
            double frontRightPower = Range.clip(driveRight - strafe, -1.0, 1.0);
            double backRightPower = Range.clip(driveRight + strafe, -1.0, 1.0);

            ////////////////////////////////////////////

            // Send calculated power to motors
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


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
