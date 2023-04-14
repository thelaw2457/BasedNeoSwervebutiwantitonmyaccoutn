// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

/** Add your docs here. */
public class AutoPathHelper {
    public static Command followPath(
        final Swerve drive, final String pathName, final HashMap<String, Command> eventMap) {
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drive::getPose,
                drive::resetOdometry,
                Constants.kSwerve.KINEMATICS,
                new PIDConstants(6, 0, 0),
                new PIDConstants(10, 0, 0), 
                drive::setModuleStates,
                eventMap,
                drive
            );
            return autoBuilder.fullAuto(
                PathPlanner.loadPathGroup(pathName, new PathConstraints(2.5, 1.5)));
        }}
