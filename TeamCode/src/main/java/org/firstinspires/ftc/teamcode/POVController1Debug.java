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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="POV Controller 1 Live Debug", group="DEBUG")
//@Disabled
public class POVController1Debug extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        //telemetry.addData("Status", "Initializing...");
        telemetry.addData("Status", "Debugger Initialized");
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Right Stick X", "(%.2f)", gamepad1.right_stick_x);
        telemetry.addData("Left Stick -Y", "(%.2f)", -gamepad1.left_stick_y);
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("Y Button", gamepad1.y);
        telemetry.addData("X Button", gamepad1.x);
        telemetry.addData("DPad Up", gamepad1.dpad_up);
        telemetry.addData("DPad Right", gamepad1.dpad_right);
        telemetry.addData("DPad Down", gamepad1.dpad_down);
        telemetry.addData("DPad Left", gamepad1.dpad_left);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Trigger", "(%.2f)", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", "(%.2f)", gamepad1.left_trigger);
        telemetry.addData("Right Stick Button", gamepad1.right_stick_button);
        telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
        telemetry.addData("DPad Top-Right Combo", (gamepad1.dpad_up && gamepad1.dpad_right));
        telemetry.addData("DPad Bottom-Right Combo", (gamepad1.dpad_down && gamepad1.dpad_right));
        telemetry.addData("DPad Bottom-Left Combo", (gamepad1.dpad_down && gamepad1.dpad_left));
        telemetry.addData("DPad Top-Left Combo", (gamepad1.dpad_up && gamepad1.dpad_left));
    }
    @Override
    public void stop() {
    }
}
