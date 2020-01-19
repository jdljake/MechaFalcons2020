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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ArmTester", group="Linear Opmode")

public class ArmTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftArmDrive = null;
    private DcMotorEx rightArmDrive = null;
    private Toggle tgg = new Toggle();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftArmDrive = hardwareMap.get(DcMotorEx.class, "left_arm_drive");
        rightArmDrive = hardwareMap.get(DcMotorEx.class, "right_arm_drive");

        //reset encoders during init
        leftArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        leftArmDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightArmDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArmDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightArmDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double armPower;
        armPower = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //////////gamepad 2 ////////////////////////


            if(tgg.toggle(gamepad2.dpad_up)){
                armPower += 0.1;
            }

            if(tgg.toggle(gamepad2.dpad_down)){
                armPower -= 0.1;
            }

            ////////////////////////////////////////////




            //locks the motors in place when not moving while opModeIsActive
//            if (!rightArmDrive.isBusy() && !leftArmDrive.isBusy()){
//                leftArmDrive.setTargetPosition(leftArmDrive.getCurrentPosition());
//                rightArmDrive.setTargetPosition(rightArmDrive.getCurrentPosition());
//                rightArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftArmDrive.setVelocity(200);
//                leftArmDrive.setVelocity(200);
//                armPower = 0;
//            }

            // Send calculated power to motors
            leftArmDrive.setPower(armPower);
            rightArmDrive.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", armPower);
            telemetry.addData("RightArmPos", rightArmDrive.getCurrentPosition());
            telemetry.addData("LeftArmPos", leftArmDrive.getCurrentPosition());
            telemetry.update();
            tgg.reset();
        }
    }
}
