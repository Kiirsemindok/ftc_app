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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.V.version;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Basic Tank Test TeleOp", group="Iterative Opmode")
//@Disabled
public class TestOpModeTele extends OpMode
{
    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor lift_motor = null;
    private Servo bucket = null;
    private DcMotor intake = null; //left_arm
    private DcMotor hook = null; //lift_arm
    private double intakeOn = 0;
    private double hookPower = 0;
    private boolean gp1unlocked = false;
    private boolean gp2unlocked = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_drive_back"); // Motor 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive"); // Motor 0
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive"); // Motor 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_drive_back"); // Motor 3
        bucket = hardwareMap.get(Servo.class, "bucket"); // Servo 2
        lift_motor = hardwareMap.get(DcMotor.class, "lift_motor"); //Motor 0
        intake = hardwareMap.get(DcMotor.class, "left_arm"); //Motor 1
        hook = hardwareMap.get(DcMotor.class, "lift_arm"); //Motor 2
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        lift_motor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        hook.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", version.tversion);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double liftPower;
        // check to see if we need to move the servo.
        if (gamepad1.left_stick_button && gamepad1.right_stick_button && !gp1unlocked) {
            gp1unlocked = true;
        } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && gp1unlocked) {
            gp1unlocked = false;
        }
        if (gamepad2.left_stick_button && gamepad2.right_stick_button && !gp2unlocked) {
            gp2unlocked = true;
        } else if (gamepad2.left_stick_button && gamepad2.right_stick_button && gp2unlocked) {
            gp2unlocked = false;
        }

        if (gamepad2.y) {
            //Move straight up
            bucket.setPosition(0.4);
        } else if (gamepad2.a) {
            //Move to 90 degrees
            bucket.setPosition(0.5);
        } else if (gamepad2.b) {
            //Move to 180 degrees.
            bucket.setPosition(1);
        } else if (gamepad2.x) {
            //Move to 0 degrees
            bucket.setPosition(0);
        }

        if (gamepad1.y && gp1unlocked) {
            //Move straight up
            bucket.setPosition(0.4);
        } else if (gamepad1.a && gp1unlocked) {
            //Move to 90 degrees
            bucket.setPosition(0.5);
        } else if (gamepad1.b && gp1unlocked) {
            //Move to 180 degrees.
            bucket.setPosition(1);
        } else if (gamepad1.x && gp1unlocked) {
            //Move to 0 degrees
            bucket.setPosition(0);
        }
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // double drive = -gamepad1.left_stick_y;
        // double turn  =  gamepad1.right_stick_x;
        // leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        // rightPower   = Range.clip(drive - turn, -1.0, 1.0);
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        if (gamepad2.right_bumper && (intakeOn == 0)) {
            intakeOn = -1;
        } else if (gamepad2.right_bumper && (intakeOn == -1)) {
            intakeOn = 0;
        } else if (gamepad2.left_bumper && (intakeOn == 1)) {
            intakeOn = -1;
        } else if (gamepad2.left_bumper && (intakeOn == -1)) {
            intakeOn = 1;
        } else if (gamepad2.left_bumper && (intakeOn == 1)) {
            intakeOn = 0;
        } else if (gamepad2.left_bumper && (intakeOn == 0)) {
            intakeOn = 1;
        }
        // Send calculated power to wheels
        leftPower = gamepad1.left_stick_y;
        leftFrontDrive.setPower(-leftPower);
        leftBackDrive.setPower(leftPower);
        rightPower = gamepad1.right_stick_y;
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
        if (gp2unlocked) {
            leftPower = gamepad2.left_stick_y;
            leftFrontDrive.setPower(-leftPower);
            leftBackDrive.setPower(leftPower);
            rightPower = gamepad2.right_stick_y;
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);
        }
        liftPower = gamepad2.right_trigger;
        lift_motor.setPower(liftPower);
        liftPower = -gamepad2.left_trigger;
        lift_motor.setPower(liftPower);
        if (gp1unlocked) {
            liftPower = gamepad2.right_trigger;
            lift_motor.setPower(liftPower);
            liftPower = -gamepad2.left_trigger;
            lift_motor.setPower(liftPower);
        }

        if (gamepad2.dpad_up) {
            hookPower = 0.25;
        } else if (gamepad2.dpad_down) {
            hookPower = -1;
        } else if (!gp1unlocked && (gamepad2.dpad_down || gamepad2.dpad_up)) {
            hookPower = 0;
        }
        hook.setPower(hookPower);
        if (gamepad1.dpad_up && gp1unlocked) {
            hookPower = 0.25;
        } else if (gamepad1.dpad_down && gp1unlocked) {
            hookPower = -1;
        } else if (gp1unlocked && !(gamepad2.dpad_down || gamepad2.dpad_up)) {
            hookPower = 0;
        }
        hook.setPower(hookPower);

        while (gamepad2.right_bumper || gamepad2.left_bumper) {
            sleep(500);
            if (gamepad2.right_bumper && (intakeOn == 0)) {
                intakeOn = -1;
            } else if (gamepad2.right_bumper && (intakeOn == -1)) {
                intakeOn = 0;
            } else if (gamepad2.left_bumper && (intakeOn == 1)) {
                intakeOn = -1;
            } else if (gamepad2.left_bumper && (intakeOn == -1)) {
                intakeOn = 1;
            } else if (gamepad2.left_bumper && (intakeOn == 1)) {
                intakeOn = 0;
            } else if (gamepad2.left_bumper && (intakeOn == 0)) {
                intakeOn = 1;
            }
        }
        intake.setPower(intakeOn);

        while ((gamepad1.right_bumper || gamepad1.left_bumper) && gp1unlocked) {
            sleep(500);
            if (gamepad1.right_bumper && (intakeOn == 0) && gp1unlocked) {
                intakeOn = -1;
            } else if (gamepad1.right_bumper && (intakeOn == -1) && gp1unlocked) {
                intakeOn = 0;
            } else if (gamepad1.left_bumper && (intakeOn == 1) && gp1unlocked) {
                intakeOn = -1;
            } else if (gamepad1.left_bumper && (intakeOn == -1) && gp1unlocked) {
                intakeOn = 1;
            } else if (gamepad1.left_bumper && (intakeOn == 1) && gp1unlocked) {
                intakeOn = 0;
            } else if (gamepad1.left_bumper && (intakeOn == 0) && gp1unlocked) {
                intakeOn = 1;
            }
        }
        intake.setPower(intakeOn);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Version", version.tversion);
        telemetry.addData("Controller 1 Unlocked", gp1unlocked);
        telemetry.addData("Controller 2 Unlocked", gp2unlocked);
        telemetry.addData("Left Front Power", leftFrontDrive.getPower());
        telemetry.addData("Left Back Power", leftBackDrive.getPower());
        telemetry.addData("Right Front Power", rightFrontDrive.getPower());
        telemetry.addData("Right Back Power", rightBackDrive.getPower());
        telemetry.addData("Lift Motor Power", lift_motor.getPower());
        telemetry.addData("Bucket Servo Position", bucket.getPosition());
        telemetry.addData("Hook Power", hook.getPower());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
