// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.RecalibrateElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevatorSmart;
import frc.robot.commands.IntakeArm.GoToAngleSmart;
import frc.robot.commands.IntakeArm.RecalibrateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighAuto extends SequentialCommandGroup {
  /** Creates a new CubeAuto. */
  public ScoreHighAuto(Arm arm, Elevator elevator, RollerIntake rollerIntake, Drivetrain drivetrain, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ReleaseArm(arm),
      new RecalibrateArm(arm),
      new ParallelRaceGroup(new GoToAngleSmart(arm, 63), new ExtendElevatorSmart(elevator, -59), new ParallelCommandGroup(new WaitCommand(2.85), new RunIntakeIn(rollerIntake))),
      new ParallelCommandGroup(new MoveForTime(drivetrain, 0.6, true), new RunIntakeIn(rollerIntake)), //MAKE SAME TIME AS BACK
      new WaitCommand(0.1),
      new RunIntakeOut(rollerIntake),
      new MoveForTime(drivetrain, 0.6, false), //MAKE SAME TIME AS FORWARD
      new ParallelCommandGroup(new RecalibrateArm(arm), new RecalibrateElevator(elevator)),
      new MoveForTime(drivetrain, time, false, 0.6)
    );
  }
}
