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

import java.io.PrintWriter;
import java.io.StringWriter;

// Drive loop copied from the FTC SDK sample BasicOmniOpMode_Linear.
// Uses our shared hardware initialization and adds intake, shooter, transfer, and flower ramp controls.
@TeleOp(name="TeleOp Red", group="FTC10820")
public class TeleOpRed extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize();
        //if (shooterMotor != null) {
        //    shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //}
        updateShooterTelemetry(false, SHOOTER_START_RPM);

        boolean shooterRunning = false;
        double shooterRpm = SHOOTER_START_RPM;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;
        boolean stopperOpen = false;
        boolean yWasPressed = false;
        boolean rightBumperWasPressed = false;
        boolean flowerRampAtPositionHalf = true;
        boolean leftBumperWasPressed = false;
        boolean setVelocityTrigger = false;

        waitForStart();
        if (isStopRequested()) {
            return;
        }

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
                    setVelocityTrigger = true ;
                }

                rightBumperWasPressed = rightBumperPressed;

                // Controller 2 changes shooter RPM by 50 once per D-pad press.
                boolean dpadUpPressed = gamepad2.dpad_up;
                boolean dpadDownPressed = gamepad2.dpad_down;
                if (dpadUpPressed && !dpadUpWasPressed) {
                    shooterRpm += SHOOTER_RPM_STEP;
                    setVelocityTrigger = true;
                }
                if (dpadDownPressed && !dpadDownWasPressed) {
                    shooterRpm = Math.max(SHOOTER_MIN_RPM, shooterRpm - SHOOTER_RPM_STEP);
                    setVelocityTrigger = true;
                }
                dpadUpWasPressed = dpadUpPressed;
                dpadDownWasPressed = dpadDownPressed;

                transferMotor.setPower(shooterRunning ? 1.0 : 0.0);
                if ((shooterMotor != null) && (setVelocityTrigger)) {
                    // 28 motor encoder ticks/rev * 2 motor turns per shooter turn = 56.
                    shooterMotor.setVelocity(shooterRunning ? shooterRpm * 56.0 / 60.0 : 0.0);
                    //shooterMotor.setVelocity(shooterRunning?1000:0);
                    setVelocityTrigger = false ;
                }

                // Toggle the stopper once per gamepad 2 Y press, independently of the shooter.
                boolean yPressed = gamepad2.y;
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

                updateShooterTelemetry(shooterRunning, shooterRpm);
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

    private void updateShooterTelemetry(boolean shooterRunning, double shooterRpm) {
        telemetry.clearAll();
        telemetry.addData("Shooter / transfer", shooterRunning ? "On" : "Off");
        telemetry.addData("Shooter selected (flywheel RPM)", "%.0f", shooterRpm);
        telemetry.addData("Shooter commanded (flywheel RPM)", "%.0f",
                shooterRunning ? shooterRpm : 0.0);
        telemetry.addData("G2 shooter D-pad", "up=%s down=%s",
                gamepad2.dpad_up, gamepad2.dpad_down);
        if (shooterMotor != null) {
            telemetry.addData("Shooter RPM", "%.0f", shooterMotor.getVelocity() * 60.0 / 56.0);
        } else {
            telemetry.addData("Shooter RPM", "Unavailable");
        }
        telemetry.update();
    }
}
