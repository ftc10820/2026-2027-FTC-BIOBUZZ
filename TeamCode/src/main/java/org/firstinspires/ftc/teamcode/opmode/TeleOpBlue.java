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
// Uses our shared hardware initialization and adds intake, shooter, transfer, and flower ramp controls.
@TeleOp(name="TeleOp Blue", group="FTC10820")
public class TeleOpBlue extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize();

        boolean shooterRunning = false;
        boolean stopperOpen = false;
        boolean yWasPressed = false;
        double shooterTargetRpm = SHOOTER_TARGET_RPM;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;
        boolean leftBumperWasPressed = false;
        boolean flowerRampAtPositionHalf = false;
        boolean xWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            try {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                // Road Runner uses positive left strafe and counterclockwise rotation.
                double lateral = -gamepad1.left_stick_x;
                double yaw = -gamepad1.right_stick_x;
                axial = Math.pow(axial,3);
                lateral = Math.pow(lateral,3);
                yaw = Math.pow(yaw,3);

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

                // Hold left trigger to reverse or right trigger to run forward; reverse has priority.
                if (gamepad1.left_trigger > 0.0)
                    intakeMotor.setPower(-1.0);
                else if (gamepad1.right_trigger > 0.0)
                    intakeMotor.setPower(1.0);
                else
                    intakeMotor.setPower(0.0);

                // Toggle the shooter and transfer together once per left-bumper press.
                boolean leftBumperPressed = gamepad1.left_bumper;
                if (leftBumperPressed && !leftBumperWasPressed) {
                    shooterRunning = !shooterRunning;
                }
                leftBumperWasPressed = leftBumperPressed;

                // Adjust once per D-pad press; holding a button does not repeat.
                boolean dpadUpPressed = gamepad1.dpad_up;
                boolean dpadDownPressed = gamepad1.dpad_down;
                double rpmAdjustment = 0.0;
                if (dpadUpPressed && !dpadUpWasPressed) {
                    rpmAdjustment += SHOOTER_RPM_STEP;
                }
                if (dpadDownPressed && !dpadDownWasPressed) {
                    rpmAdjustment -= SHOOTER_RPM_STEP;
                }
                double previousTargetRpm = shooterTargetRpm;
                shooterTargetRpm = Math.max(0.0,
                        Math.min(SHOOTER_MAX_RPM, shooterTargetRpm + rpmAdjustment));
                if (shooterTargetRpm != previousTargetRpm) {
                    telemetry.log().add("Flywheel speed set to %.0f RPM", shooterTargetRpm);
                }
                dpadUpWasPressed = dpadUpPressed;
                dpadDownWasPressed = dpadDownPressed;

                transferMotor.setPower(shooterRunning ? 1.0 : 0.0);
                if (shooterMotor != null) {
                    shooterMotor.setVelocity(shooterRunning
                            ? shooterTargetRpm * SHOOTER_TICKS_PER_REV / 60.0 : 0.0);
                }

                // Toggle the stopper once per Y press, independently of the shooter.
                boolean yPressed = gamepad1.y;
                if (yPressed && !yWasPressed) {
                    stopperOpen = !stopperOpen;
                    stopperServo.setPosition(stopperOpen
                            ? STOPPER_OPEN_POSITION : STOPPER_CLOSED_POSITION);
                }
                yWasPressed = yPressed;

                // Toggle once per X press, even when the button is held down.
                boolean xPressed = gamepad1.x;
                if (xPressed && !xWasPressed) {
                    flowerRampAtPositionHalf = !flowerRampAtPositionHalf;
                    flowerRampServo.setPosition(flowerRampAtPositionHalf ? 0.5 : 0.0);
                }
                xWasPressed = xPressed;
                telemetry.addData("Flywheel target RPM", shooterTargetRpm);
                telemetry.addData("Shooter / transfer", shooterRunning ? "On" : "Off");
                telemetry.update();
            } catch (Exception e) {
              handleExpection(e);
            }
        }

        drive.leftFront.setPower(0.0);
        drive.rightFront.setPower(0.0);
        drive.leftBack.setPower(0.0);
        drive.rightBack.setPower(0.0);
        intakeMotor.setPower(0.0);
        stopperServo.setPosition(STOPPER_CLOSED_POSITION);
        transferMotor.setPower(0.0);
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0.0);
        }
    }
}
