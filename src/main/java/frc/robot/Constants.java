// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final int FRONTLEFT = 4;
    public static final int FRONTRIGHT = 1;
    public static final int BACKLEFT = 3;
    public static final int BACKRIGHT = 2;
    
    public static final double kDistancePerWheelRevolutionMeters = Units.inchesToMeters(Math.PI * 6.0);
    public static final double kGearReduction = 10.71;
    public static final double ksVolts = 0.090264;
    public static final double kvVoltSecondsPerMeter = 2.5998;
    public static final double kaVoltSecondsSquaredPerMeter = 0.8777;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(23));
    public static final double kPDriveVel = 0; //kp = 100.66
  }
  public static final class AutoConstants {
    public static final double kRamseteB = 2D;
    public static final double kRamseteZeta = 0.7D;
  }

  public static final class JoystickConstants {
    // Controllers
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    // XboxOne Joysticks (axes)
    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_STICK_X = 4;
    public static final int RIGHT_STICK_Y = 5;

    // XboxOne Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int LOGO_LEFT = 7;
    public static final int LOGO_RIGHT = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;
  }
}
