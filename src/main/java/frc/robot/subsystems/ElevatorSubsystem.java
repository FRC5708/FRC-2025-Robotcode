// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase {
  public SparkMax elevatorMotor1 = new SparkMax(Constants.NonChassis.elevatorMotorID1, MotorType.kBrushless);
  public SparkMax elevatorMotor2 = new SparkMax(Constants.NonChassis.elevatorMotorID2, MotorType.kBrushless);
  private RelativeEncoder encoder1 = elevatorMotor1.getEncoder();
  private RelativeEncoder encoder2 = elevatorMotor2.getEncoder();

  private PIDController m_pid = new PIDController(0.04, 0,0.01);

  private double targetPos = 0.0;
  private boolean manualMode;
  // private int desiredPoint = 0;
  // private double driveSpeed = 0.5;

  DigitalInput input = new DigitalInput(1);

  public ElevatorSubsystem() {
    m_pid.setTolerance(0.3);
    resetEncoder();
    elevatorStop();
    manualMode = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setSpeed(double speed){
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed); //This way you can set them both at once 
  } 
  
  public void elevatorUp(double power) {
    if (input.get()) {
      setSpeed(power);
      System.out.println("Elevator Up Power: " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorUpFast(double power) {
    if (input.get()) {
      setSpeed(power);
      System.out.println("Elevator Up Power Motor; " + power);
    } else {
      System.out.println("Elevator Blocked By Coral");

    }
  }

  public void elevatorDown(double power) {
    setSpeed(power);
    System.out.println("Elevator Down Power: " + power);
  }

  public void elevatorHold() {
    setSpeed(0.03);
    System.out.println("Elevator Hold Power: .03");
  }

  public void elevatorStop()
  {
    setSpeed(0);
    targetPos = calculatePID();
  }

  public boolean checkPhotoeye() {
    SmartDashboard.putNumber("encoder position Motor 1", getPosition());
    SmartDashboard.putNumber("encoder position Motor 2", getPositionEncoder2());
    SmartDashboard.putBoolean("Photoeye", input.get());
    return input.get();
  }

  public void moveToL0(){
    targetPos = 0;
  }

  public void moveToL1(){
    targetPos = Constants.NonChassis.ticksToL1;
  }

  public void moveToL2(){
    targetPos = Constants.NonChassis.ticksToL2;
  }
  
  public void moveToL3(){
    targetPos = Constants.NonChassis.ticksToL3;
  }

  public void moveToL4(){
    targetPos = Constants.NonChassis.ticksToL4;
  }

public void resetEncoder() {
  encoder1.setPosition(0.2);
  encoder2.setPosition(0.2);
}

public double getPosition() {
  return encoder1.getPosition();
}

public double getPositionEncoder2() {
  return encoder2.getPosition();
}

public void travelToSetpoint() {
  if (!manualMode){
    setSpeed(m_pid.calculate(getPosition(), targetPos));
  }
  else {
    elevatorHold();
  }
}

public void manualModeToggle() {
  manualMode = !manualMode;
}

  public double calculatePID() {
    return m_pid.calculate(getPosition());
  }

}


  // public void setTarget(int target) {
  //   if (atPoint() == 0) {
  //     driveSpeed = 0.01;
  //   }
  //   if ((target < 4 && target > -1) | target == 10) {
  //     desiredPoint = target;
  //   }
  //   m_pid.setSetpoint(0.2);
  //   speed = 0.02;
  //   if (desiredPoint == 1) {
  //     m_pid.setSetpoint(Constants.NonChassis.ticksToL2);
  //     speed = 0.05;
  //   } else if (desiredPoint == 2) {
  //     m_pid.setSetpoint(Constants.NonChassis.ticksToL3);
  //     speed = 0.06;
  //   } else if (desiredPoint == 3) {
  //     m_pid.setSetpoint(Constants.NonChassis.ticksToL4);
  //     speed = 0.07;
  //   }
  // }

  // public int getTarget() {
  //   return desiredPoint;
  // }

  // public double getPosition() {
  //   return encoder1.getPosition();
  // }

  // public double getPositionEncoder2() {
  //   return encoder2.getPosition();
  // }

  // public int atPoint() {
  //   SmartDashboard.putNumber("desired point", desiredPoint);
  //   SmartDashboard.putNumber("encoder position", getPosition());
  //   double targetTicks = 0.1;
  //   if (desiredPoint == 1) {
  //     targetTicks = Constants.NonChassis.ticksToL2;
  //   } else if (desiredPoint == 2) {
  //     targetTicks = Constants.NonChassis.ticksToL3;
  //   } else if (desiredPoint == 3) {
  //     targetTicks = Constants.NonChassis.ticksToL4;
  //   } else if (desiredPoint == 10) {
  //     targetTicks = Constants.NonChassis.ticksShootL1;
  //   }
  //   if (encoder1.getPosition() > targetTicks + 1.5) {
  //     return -1;
  //   } else if (encoder1.getPosition() < targetTicks - 1.5) {
  //     return 1;
  //   } else {
  //     return 0;
  //   }
  // }

  // public boolean getCooked() {
  //   return cooked;
  // }

  // public double calculatePID() {
  //   return m_pid.calculate(getPosition());
  // }

  // public void cook() {
  //   cooked = !cooked;
  // }

  // public double getSpeed() {
  //   return speed;
  // }

  // public void setSpeed(double newSpeed) {
  //   speed = newSpeed;
  // }

  // public double getDriveSpeed() {
  //   return driveSpeed;
  // }

  // public void setDriveSpeed(double newSpeed) {
  //   driveSpeed = newSpeed;
  // }
