/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


class ColorSensorDriverBlueAutoOp extends LinearOpMode {

  public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};

  public ColorSensorDevice device = ColorSensorDevice.HITECHNIC_NXT;

  ColorSensor colorSensor;
  DeviceInterfaceModule cdim;
  LED led;
  TouchSensor t;
  DcMotor motorRightOne;
  DcMotor motorRightTwo;
  DcMotor motorLeftOne;
  DcMotor motorLeftTwo;


  @Override
  public void runOpMode() throws InterruptedException {
    hardwareMap.logDevices();

    switch (device) {
      case HITECHNIC_NXT:
        colorSensor = hardwareMap.colorSensor.get("c1");
        break;
    }
    motorRightOne = hardwareMap.dcMotor.get("m1");
    motorRightTwo = hardwareMap.dcMotor.get("m2");
    motorLeftOne = hardwareMap.dcMotor.get("m3");
    motorLeftTwo = hardwareMap.dcMotor.get("m4");




    waitForStart();

    float hsvValues[] = {0,0,0};
    final float values[] = hsvValues;
    int block;
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    while (opModeIsActive()) {


      switch (device) {
        case HITECHNIC_NXT:
          Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
          break;

      }
      if (colorSensor.equals(colorSensor.blue())) {
      } else {
        if (time < 1.0) {
          motorRightOne.setPower(1.0);
          motorRightTwo.setPower(1.0);
          motorLeftOne.setPower(-1.0);
          motorLeftTwo.setPower(-1.0);
        }
      }

      telemetry.addData("Blue ", colorSensor.blue());


      waitOneFullHardwareCycle();
    }
  }


}