/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Drive loop copied from the FTC SDK sample BasicOmniOpMode_Linear.
// Uses our shared hardware initialization and adds intake and shooter controls.
@TeleOp(name="TeleOp Blue", group="FTC10820")
public class TeleOpBlue extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize();

        boolean shooterRunning = false;
        boolean leftBumperWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            try {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

                // Hold left trigger to reverse or right trigger to run forward; reverse has priority.
                if (gamepad1.left_trigger > 0.0)
                    intakeMotor.setPower(-1.0);
                else if (gamepad1.right_trigger > 0.0)
                    intakeMotor.setPower(1.0);
                else
                    intakeMotor.setPower(0.0);

                // Toggle once per press, even when the bumper is held down.
                boolean leftBumperPressed = gamepad1.left_bumper;
                if (leftBumperPressed && !leftBumperWasPressed) {
                    shooterRunning = !shooterRunning;
                }
                leftBumperWasPressed = leftBumperPressed;

                if (shooterMotor != null) {
                    shooterMotor.setVelocity(shooterRunning ? SHOOTER_TARGET_TICKS_PER_SECOND : 0.0);
                }
            } catch (Exception e) {
              handleExpection(e);
            }
        }

        drive.leftFront.setPower(0.0);
        drive.rightFront.setPower(0.0);
        drive.leftBack.setPower(0.0);
        drive.rightBack.setPower(0.0);
        intakeMotor.setPower(0.0);
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0.0);
        }
    }
}
