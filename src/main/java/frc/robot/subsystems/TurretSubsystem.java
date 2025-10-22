// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  private SparkMax rotationMotor;
  private SparkFlex leftPivotMotor;
  private SparkFlex rightPivotMotor;
  
  public TurretSubsystem() {
    rotationMotor = new SparkMax(11, MotorType.kBrushless);
    leftPivotMotor = new SparkFlex(12, MotorType.kBrushless);
    rightPivotMotor = new SparkFlex(13, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
