// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiece extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */
  public TwoPiece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    // move arm up
    // roller out
    // recalibrate arm
    // move backward to piece
    // turn 180
    // with arm down - roller in
    // turn 180
    // move for some distance
    // arm up
    // roller out
    addCommands();
  }
}
