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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;


/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Hafza's comments:
 * all we really need are the Tx value and the YAW
 *Tx: for the PID loop that is gonna control servo power and rotation to align with the April tag
 *YAW: using botpose we can find roll, pitch, yaw, then use these degrees to see mounting orientation
 *  IT WILL BE FIELD CENTRIC THO!
 *positive Tx is left
 * for continuous power = error * kP
 */
@Configurable
@TeleOp(name = "AprilTagEasy")
public class AprilTagEasy extends LinearOpMode {

    // Change this to Limelight3A
    private Limelight3A limelight;
    private Servo raxon;

    private Servo laxon;
    private double raxonPos;
    private double laxonPos;


    @Override
    public void runOpMode() {

        // Initialize Limelight BEFORE waitForStart()
        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Switch to AprilTag pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            raxon.setPosition(.6094);
            laxon.setPosition(.6094);

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        limelight.close();

    }   // end method runOpMode()

    private void telemetryAprilTag() {
        // Check if limelight exists
        if (limelight == null) {
            telemetry.addData("Error", "Limelight not initialized");
            return;
        }

        // Get results from Limelight (not AprilTagProcessor!)
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No valid targets");
            return;
        }

        // Get fiducial (AprilTag) results from Limelight
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("# AprilTags Detected", fiducials.size());

        // Step through the list of detections and display info for each one
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            telemetry.addLine(String.format(Locale.getDefault(),
                    "\n==== (ID %d)", fiducial.getFiducialId()));

            telemetry.addLine(String.format(Locale.getDefault(),
                    "Target X: %.2f deg, Y: %.2f deg",
                    fiducial.getTargetXDegrees(),
                    fiducial.getTargetYDegrees()));

            telemetry.addLine(String.format(Locale.getDefault(),
                    "Robot Pose: X=%.2f, Y=%.2f, Z=%.2f",
                    fiducial.getRobotPoseFieldSpace().getPosition().x,
                    fiducial.getRobotPoseFieldSpace().getPosition().y,
                    fiducial.getRobotPoseFieldSpace().getPosition().z));

        }

        telemetry.addLine("\nNote: Using Limelight API for AprilTag detection");

    }   // end method telemetryAprilTag()

}   // end class