// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.revrobotics.SparkMaxPIDController;

/** Add your docs here. */
public class PIDGains {
    public final double _p;
    public final double _i;
    public final double _d;

    public PIDGains(double p, double i, double d) {
        _p = p;
        _i = i;
        _d = d;
    }

    public static void setSparkMaxGains(SparkMaxPIDController _controller, PIDGains _gains) {
        _controller.setP(_gains._p);
        _controller.setI(_gains._i);
        _controller.setD(_gains._d);
    }
}
