// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final CANSparkMax leftMotor = new CANSparkMax(DriveConstants.leftMotor, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(DriveConstants.rightMotor, MotorType.kBrushless);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;


  public DriveTrain() {
    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(DriveConstants.kLeftInvert);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(DriveConstants.kRightInvert);
    drive.setMaxOutput(DriveConstants.kMaxOutput);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetDrivetrainEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getPosition() * DriveConstants.kDistancePerPulseFactor;
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getPosition() * DriveConstants.kDistancePerPulseFactor;
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity() * DriveConstants.kDistancePerPulseFactor / (double)60.00;
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity() * DriveConstants.kDistancePerPulseFactor / (double)60.00;
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance())/(double)2.00;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
    System.out.println(getAverageEncoderDistance());
  }

}
