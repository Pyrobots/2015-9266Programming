/*
 * Copyright (c) 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.ftcrobotcontroller.opmodes;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;


public class ColorSensorAutoOp extends OpMode {

  // how long to hold before the next action
  final static double HOLD_POSITION = 3.0; // in seconds

  // wheel speed
  final static double MOTOR_POWER = 1.0; //
  final static double sta = 5.0;// scale from 0 to 1

  // Turn around at least twice in 20 seconds.
  private double turnTime = 10.0;



  // when paused time as passed, we will switch back to measurement mode.
  double pauseTime = 1.0;

  DcMotor motorRightOne;
  DcMotor motorRightTwo;
  DcMotor motorLeftOne;
  DcMotor motorLeftTwo;
  ColorSensor color;

  @Override
  public void init() {
    motorRightOne = hardwareMap.dcMotor.get("m1");
    motorRightTwo = hardwareMap.dcMotor.get("m2");
    motorLeftOne = hardwareMap.dcMotor.get("m3");
    motorLeftTwo = hardwareMap.dcMotor.get("m4");
    color = hardwareMap.colorSensor.get("c1");
  }

  @Override
  public void init_loop() {


    // Set the compass to calibration mode
    telemetry.addData("Color", "Color in calibration mode");

    // calculate how long we should hold the current position
  }

  @Override
  public void loop() {
  if(color.equals(color.blue()))
    // make sure pauseTime has passed before we take any action
    if (time < pauseTime) {

      // have we turned around at least twice in 20 seconds?
      motorRightOne.setPower(MOTOR_POWER);
      motorRightTwo.setPower(MOTOR_POWER);
      motorLeftOne.setPower(-MOTOR_POWER);
      motorLeftTwo.setPower(-MOTOR_POWER);

    }

    else if (pauseTime < 2.0 )
    {
      motorRightOne.setPower(-MOTOR_POWER);
      motorRightTwo.setPower(-MOTOR_POWER);
      motorLeftOne.setPower(MOTOR_POWER);
      motorLeftTwo.setPower(MOTOR_POWER);
    }

    else if(2.0 < 3.0)
    {
      motorRightOne.setPower(0);
      motorRightTwo.setPower(0);
      motorLeftOne.setPower(0);
      motorLeftTwo.setPower(0);
    }
  }


}
