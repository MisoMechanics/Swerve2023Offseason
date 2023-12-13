// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team9442.frc2024.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team9442.frc2024.constants.ElevatorConstants;
import team9442.frc2024.constants.DrivetrainConstants;
import team9442.frc2024.constants.GlobalConstants;
import team9442.frc2024.subsystems.Drivetrain;
import team9442.frc2024.subsystems.Elevator;
import team9442.frc2024.util.Joysticks;

public class RobotContainer {

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        elevator.setDefaultCommand(elevator.holdAtCall());
        drivetrain.setDefaultCommand(
                drivetrain.drive(
                        () -> -DrivetrainConstants.kXDriveLimiter.calculate(mainController.getLeftStickX()),
                        () -> -DrivetrainConstants.kYDriveLimiter.calculate(mainController.getLeftStickY()),
                        () ->
                                -DrivetrainConstants.kThetaDriveLimiter.calculate(
                                        mainController.getRightStickX()),
                        true,
                        false));
    }

    private void configureBindings() {
        coController.buttonA.onTrue(elevator.length(ElevatorConstants.kMinimumPosition));
        coController.buttonB.onTrue(elevator.length(ElevatorConstants.kMidPosition));
        coController.buttonX.onTrue(elevator.home(-0.2));
        coController
                .leftBumper
                .whileTrue(elevator.openloop(() -> coController.getLeftStickX() * 0.5));

        mainController.start.onTrue(drivetrain.zeroGyroCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private final Joysticks mainController = new Joysticks(0, DrivetrainConstants.kAxisDeadzone);
    private final Joysticks coController = new Joysticks(1, DrivetrainConstants.kAxisDeadzone);

    private final Drivetrain drivetrain =
            new Drivetrain(
                    DrivetrainConstants.kModules, 
                    DrivetrainConstants.kGyro, 
                    DrivetrainConstants.kSwerveKinematics, 
                    DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                    DrivetrainConstants.kMaxAngularRadiansPerSecond);

    private final Elevator elevator =
            new Elevator(
                    ElevatorConstants.kMaster,
                    ElevatorConstants.kFollower,
                    ElevatorConstants.kElevatorServoConstants,
                    GlobalConstants.kDt);
}
