// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  
  private SparkMax upperIntakeMotor;

  private TalonFX topFlywheelMotor;
  private TalonFX leftFlywheelMotor;
  private TalonFX rightFlywheelMotor;

  private DigitalInput shooterPhotoElectric;

  public ShooterSubsystem() {
    upperIntakeMotor = new SparkMax(10, MotorType.kBrushless);

    topFlywheelMotor = new TalonFX(14);
    leftFlywheelMotor = new TalonFX(15);
    rightFlywheelMotor = new TalonFX(16);

    shooterPhotoElectric = new DigitalInput(1);
  }

  public void runUpperIntake(double speed) {
    upperIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
