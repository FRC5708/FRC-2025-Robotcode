// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralSubsystem extends SubsystemBase {
  // public SparkMax intakeMotor1 = new SparkMax(Constants.NonChassis.coralIntakeMotorID1, MotorType.kBrushless);
  // public SparkMax intakeMotor2 = new SparkMax(Constants.NonChassis.coralIntakeMotorID2, MotorType.kBrushless);
  public SparkMax placeMotor3 = new SparkMax(Constants.NonChassis.coralPlaceMotorID3, MotorType.kBrushless);
  public SparkMax placeMotor4 = new SparkMax(Constants.NonChassis.coralPlaceMotorID4, MotorType.kBrushless);

  public CoralSubsystem() {
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void coralIntake(double power) {
  //   intakeMotor1.set(power);
  //   intakeMotor2.set(-power);
  //   System.out.println("Coral Intake Power: " + power);
  // }

  // public void coralReverseIntake(double power) {
  //   intakeMotor1.set(power);
  //   intakeMotor2.set(-power);
  //   System.out.println("Coral Outtake Power: " + power);
  // }

  public void coralPlace(double power) {
    placeMotor3.set(power);
    placeMotor4.set(-power);
    //System.out.println("Coral Placing Power: " + power);
  }

  public void coralReversePlace(double power) {
    placeMotor3.set(power);
    placeMotor4.set(-power);
    //System.out.println("Coral Reverse Placing Power: " + power);
  }

  // public void intakeAndSpit(double pow1,double pow2) {
  //   placeMotor3.set(pow1);
  //   placeMotor4.set(-pow1);
  //   intakeMotor1.set(pow2);
  //   intakeMotor2.set(-pow2);
  // }
}
