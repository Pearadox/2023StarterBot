// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDGains;

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
    public static final int kDriverController = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmManualDeadband = 0.05;
    public static final double kArmManualScale = 0.5;
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

    public static final boolean kFrontLeftInverted = true;
    public static final boolean kFrontRightInverted = false;
    public static final boolean kRearLeftInverted = true;
    public static final boolean kRearRightInverted = false;

    public static final int kCurrentLimit = 55;

    public static final double kTurningScale = 0.5;
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

  public final static class Arm {
    public static final int kArmCanId = 5;
    public static final boolean kArmInverted = false;
    public static final int kCurrentLimit = 40;

    public static final double kSoftLimitReverse = 0.0;
    public static final double kSoftLimitForward = 4.6;

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0); 
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 3.05;
    public static final double kIntakePosition = 4.52;
    public static final double kFeederPosition = 2.95;
  }

  public static final class Gripper {
    public static final int kGripperCanId = 6;
    public static final double kSoftLimitReverse = -34.0;
    public static final double kSoftLimitForward = 5.0;
    public static final double kClosePosition = 0.0;
    public static final double kOpenPosition = -34.0;
    public static final double kSafePosition = -29.0;
    public static final int kCurrentLimit = 10;
    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
  }
}
