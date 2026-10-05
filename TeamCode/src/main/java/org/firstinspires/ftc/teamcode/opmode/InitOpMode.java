package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public abstract class InitOpMode extends LinearOpMode {
    public MecanumDrive drive;
    public  DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotor intakeMotor;

    public void initialize() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        frontLeftDrive = drive.leftFront;
        frontRightDrive = drive.rightFront;
        backLeftDrive = drive.leftBack;
        backRightDrive = drive.rightBack;
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        System.out.println("Drive motors initialized");
        telemetry.addLine("Drive motors initialized");
        telemetry.update();

    }

    public void endOpMode() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}
