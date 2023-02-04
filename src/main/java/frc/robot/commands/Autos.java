// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase TestAuto(DriveTrain drivetrain) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TestStraight", false, new PathConstraints(0.5, 0.5));
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("IntakeDown", new AutoIntakeDown(_intake));
    // eventMap.put("IntakeIn", new AutoIntakeIn(_intake));
    // eventMap.put("IntakeStop", new IntakeStop(_intake));
    // eventMap.put("IntakeOut", new AutoIntakeOut(_intake));

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        drivetrain::getPose,
        drivetrain::resetOdometry,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        DriveTrainConstants.kDriveKinematics,
        new SimpleMotorFeedforward(
            DriveTrainConstants.ksVolts,
            DriveTrainConstants.kvVoltSecondsPerMeter,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        drivetrain::getWheelSpeeds,
        new PIDConstants(DriveTrainConstants.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts,
        eventMap,
        true,
        drivetrain

    );
    return autoBuilder.fullAuto(pathGroup);
  }
}
