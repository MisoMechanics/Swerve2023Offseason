// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team9442.frc2024.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team9442.frc2024.constants.ElevatorConstants;
import team9442.frc2024.constants.GlobalConstants;
import team9442.frc2024.subsystems.Elevator;

public class RobotContainer {

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        elevator.setDefaultCommand(elevator.holdAtCall());
    }

    private void configureBindings() {
        coController.a().onTrue(elevator.length(ElevatorConstants.kMinimumPosition));
        coController.b().onTrue(elevator.length(ElevatorConstants.kMidPosition));
        coController.x().onTrue(elevator.home(-0.2));
        coController
                .leftBumper()
                .whileTrue(elevator.openloop(() -> coController.getLeftTriggerAxis() * 0.5));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private final CommandXboxController mainController = new CommandXboxController(0);
    private final CommandXboxController coController = new CommandXboxController(1);
    private final Elevator elevator =
            new Elevator(
                    ElevatorConstants.kMaster,
                    ElevatorConstants.kFollower,
                    ElevatorConstants.kElevatorServoConstants,
                    GlobalConstants.kDt);
}
