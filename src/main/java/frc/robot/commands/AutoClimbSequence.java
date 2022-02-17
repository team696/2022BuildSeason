// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbSequence extends SequentialCommandGroup {
  Climber climber;
  Pneumatics pneumatics;
  DIOSub dioSub;
  
  /** Creates a new AutoClimbeSequence. */
  public AutoClimbSequence(Climber climber, Pneumatics pneumatics, DIOSub dioSub) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(climber, pneumatics, dioSub);
    addCommands(new AutoClimbStep1(climber, pneumatics, dioSub), new AutoClimbStep2(climber, pneumatics, dioSub), new SingleLatchRelease(climber, pneumatics));
    
  }
}
