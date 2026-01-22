/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.List;

@Configurable
@TeleOp(name = "AprilTagContinuous")
public class AprilTagContinuous extends LinearOpMode {

     Limelight3A limelight;
     CRServo raxon;
     CRServo laxon;
    AnalogInput axonEncoder;

    double ENCODER_VOLTAGE_MIN = 0.0;
    double ENCODER_VOLTAGE_MAX = 3.3;
    double ENCODER_DEGREES_PER_VOLT = 360.0 / 3.3; //need to adj
    double kP = 0.015;
    double kD = 0.003;
    double lastError = 0;
    double SMOOTHING = 0.3;
    double DEADZONE = 1.0;
    double MAX_POWER = 0.6;
    double MIN_POWER = 0.1;

    @Override
    public void runOpMode() {
        raxon = hardwareMap.get(CRServo.class, "raxon");
        laxon = hardwareMap.get(CRServo.class, "laxon");
        axonEncoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        raxon.setPower(0);
        laxon.setPower(0);

        telemetry.addLine("Continuous Servo AprilTag Tracker Ready");
        telemetry.addData("Current Position", "%.1f°", getEncoderDegrees());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            track();
            telemetry.update();
            sleep(50);
        }

        raxon.setPower(0);
        laxon.setPower(0);
        limelight.close();
    }

    private void track() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Status", "No valid camera result");
            stopServos();
            lastError = 0;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags.isEmpty()) {
            telemetry.addData("Status", "No AprilTags detected");
            stopServos();
            lastError = 0;
            return;
        }
        //horizontal
        double rawError = tags.get(0).getTargetXDegrees();

        //smoothing
        double error = (SMOOTHING * lastError) + ((1 - SMOOTHING) * rawError);

        // Calc derivative
        double derivative = error - lastError;
        lastError = error;

        // Check if we're locked in
        if (Math.abs(error) < DEADZONE) {
            stopServos();
            telemetry.addLine("✓ LOCKED ON TARGET");
            telemetry.addData("Target ID", tags.get(0).getFiducialId());
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Encoder Position", "%.1f°", getEncoderDegrees());
            return;
        }

        //PD output
        double power = (kP * error) + (kD * derivative);

        // minimum power for friction
        if (Math.abs(power) > 0.01) {
            if (power > 0) {
                power = Math.max(power, MIN_POWER);
            } else {
                power = Math.min(power, -MIN_POWER);
            }
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        laxon.setPower(power);
        raxon.setPower(power);

        telemetry.addData("Target ID", tags.get(0).getFiducialId());
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Derivative", "%.3f", derivative);
        telemetry.addData("Servo Power", "%.3f", power);
        telemetry.addData("Encoder Position", "%.1f°", getEncoderDegrees());
        telemetry.addData("Tracking...", "Adjusting turret");
    }

    private void stopServos() {
        laxon.setPower(0);
        raxon.setPower(0);
    }

    private double getEncoderDegrees() {
        double voltage = axonEncoder.getVoltage();
        // Convert voltage to degrees
        double degrees = (voltage / ENCODER_VOLTAGE_MAX) * 360.0;
        return degrees;
    }
}
