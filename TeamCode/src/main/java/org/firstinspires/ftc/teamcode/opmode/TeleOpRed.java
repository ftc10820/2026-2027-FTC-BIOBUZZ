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
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.PIDConstants;

import java.io.PrintWriter;
import java.io.StringWriter;

// Drive loop copied from the FTC SDK sample BasicOmniOpMode_Linear.
// Uses our shared hardware initialization and adds intake, shooter, transfer, and flower ramp controls.
@TeleOp(name="TeleOp Red", group="FTC10820")
public class TeleOpRed extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize();
        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        initializeTurret();

        boolean shooterRunning = false;
        double shooterTargetRpm = 900.0;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;
        boolean stopperOpen = false;
        boolean yWasPressed = false;
        boolean rightBumperWasPressed = false;
        boolean flowerRampAtPositionHalf = true;
        boolean leftBumperWasPressed = false;
        int previousTurretDirection = 0;
        long nextTurretStepTimeNs = 0;
        boolean grabberOpen = false;
        boolean grabberXWasPressed = false;

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Reapply the closed position on START as well as during INIT.
        grabberServo.setPosition(0.1);
        grabberOpen = false;
        // Require a fresh X press so a held button cannot immediately reopen it.
        grabberXWasPressed = gamepad2.x;

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

                // Toggle the shooter and transfer together once per right-bumper press.
                boolean rightBumperPressed = gamepad1.right_bumper;
                if (rightBumperPressed && !rightBumperWasPressed) {
                    shooterRunning = !shooterRunning;
                }
                rightBumperWasPressed = rightBumperPressed;

                // Adjust the shooter target once per D-pad press, including while stopped.
                boolean dpadUpPressed = gamepad2.dpad_up;
                boolean dpadDownPressed = gamepad2.dpad_down;
                if (dpadUpPressed && !dpadUpWasPressed) {
                    shooterTargetRpm += 250.0;
                }
                if (dpadDownPressed && !dpadDownWasPressed) {
                    shooterTargetRpm = Math.max(0.0, shooterTargetRpm - 250.0);
                }
                dpadUpWasPressed = dpadUpPressed;
                dpadDownWasPressed = dpadDownPressed;

                transferMotor.setPower(shooterRunning ? 1.0 : 0.0);
                if (shooterMotor != null) {
                    shooterMotor.setVelocity(shooterRunning
                            ? PIDConstants.rpmToTicks(shooterTargetRpm) : 0.0);
                }

                // Toggle the stopper once per Y press, independently of the shooter.
                boolean yPressed = gamepad1.y;
                if (yPressed && !yWasPressed) {
                    stopperOpen = !stopperOpen;
                    stopperServo.setPosition(stopperOpen
                            ? STOPPER_OPEN_POSITION : STOPPER_CLOSED_POSITION);
                }
                yWasPressed = yPressed;

                // Toggle once per left-bumper press, even when the button is held down.
                boolean leftBumperPressed = gamepad1.left_bumper;
                if (leftBumperPressed && !leftBumperWasPressed) {
                    flowerRampAtPositionHalf = !flowerRampAtPositionHalf;
                    flowerRampServo.setPosition(flowerRampAtPositionHalf ? 0.5 : 0.0);
                }
                leftBumperWasPressed = leftBumperPressed;

                // Gamepad 2: proportional slide power, left trigger up and right trigger down.
                // Releasing both triggers stops the motor; equal presses cancel each other.
                slideMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

                // Toggle grabber once per X press, even when the button is held.
                boolean grabberXPressed = gamepad2.x;
                if (grabberXPressed && !grabberXWasPressed) {
                    grabberOpen = !grabberOpen;
                    grabberServo.setPosition(grabberOpen ? 0.5 : 0.0);
                }
                grabberXWasPressed = grabberXPressed;

                // Step immediately, then every 100 ms while held. Opposite inputs cancel.
                int turretDirection = (gamepad2.dpad_left ? 1 : 0)
                        - (gamepad2.dpad_right ? 1 : 0);
                if (turretDirection != previousTurretDirection) {
                    nextTurretStepTimeNs = 0;
                }
                long nowNs = System.nanoTime();
                if (turretDirection != 0 && nowNs >= nextTurretStepTimeNs) {
                    if (Math.abs(turretTargetDegrees + turretDirection * TURRET_STEP_DEGREES)
                            <= TURRET_MAX_ANGLE_DEGREES) {
                        moveTurretByDegrees(turretDirection * TURRET_STEP_DEGREES);
                    }
                    nextTurretStepTimeNs = nowNs + TURRET_REPEAT_INTERVAL_NS;
                }
                previousTurretDirection = turretDirection;
                telemetry.addData("Turret target (deg, +left)", turretTargetDegrees);
                telemetry.addData("Shooter / transfer", shooterRunning ? "On" : "Off");
                telemetry.addData("Shooter target (RPM)", shooterTargetRpm);
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
        slideMotor.setPower(0.0);
        stopperServo.setPosition(STOPPER_CLOSED_POSITION);
        transferMotor.setPower(0.0);
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }
}
