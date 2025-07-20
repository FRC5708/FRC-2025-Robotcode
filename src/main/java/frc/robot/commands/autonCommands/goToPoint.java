// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autonCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class goToPoint extends Command {
//   /** Creates a new goToPoint. */
// private int m_point;
// private ElevatorSubsystem m_elevator;
// private double startTime;

//   public goToPoint(ElevatorSubsystem elevator, int point) {
//     m_elevator = elevator;
//     m_point = point;
//     addRequirements(m_elevator);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_elevator.setTarget(m_point);
//     startTime = System.currentTimeMillis();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     int direction = m_elevator.atPoint();
//         if(direction == 1) {
//             //m_elevator.elevatorUp(m_elevator.getSpeed());
//             m_elevator.elevatorUp(0.5);
//         }
//         else if(direction == -1) {
//             //m_elevator.elevatorDown(-m_elevator.getSpeed());
//             m_elevator.elevatorDown(-0.5);
//         }
//         else {
//             m_elevator.elevatorHold(m_elevator.getSpeed());
//         }
//         // m_elevator.elevatorUp(MathUtil.clamp(m_elevator.calculatePID(),-1,1) + 0.02);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(m_elevator.getTarget() == 0) {
//       if(m_elevator.atPoint() == 0 | System.currentTimeMillis() - startTime > 1000) {
//         m_elevator.resetEnc();
//         return true;
//       }
//     }
//     return m_elevator.atPoint() == 0;
//   }
// }
