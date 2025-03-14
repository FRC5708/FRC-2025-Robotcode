// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autonCommands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.Constants;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ElevatorDownFromL4 extends Command {
//   /** Creates a new ElevatorToL4. */
//   private ElevatorSubsystem m_elevator;
//   private double m_power;
//   private double m_startTime;
//   private double m_freaky;
//   private double timeToGo;
//   public ElevatorDownFromL4(ElevatorSubsystem elevator, double freaky) {
//     m_elevator = elevator;
//     addRequirements(m_elevator);
//     m_freaky = freaky;
//     timeToGo = Constants.NonChassis.millisDownL4;
//     if(m_freaky == 1) {
//       timeToGo -= 1000;
//     }
//     else if(m_freaky == 2) {
//       timeToGo -= 1500;
//     }
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_startTime = System.currentTimeMillis();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double power = -0.3;
//     m_elevator.elevatorDown(power);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_elevator.elevatorDown(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return System.currentTimeMillis()-m_startTime > timeToGo;
//   }
// }
