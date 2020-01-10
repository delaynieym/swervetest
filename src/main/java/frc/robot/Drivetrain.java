/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_rightFrontLocation = new Translation2d(12.75, 11);
  private final Translation2d m_leftFrontLocation = new Translation2d(-12.75, 11);
  private final Translation2d m_leftBackLocation = new Translation2d(-12.75, -11);
  private final Translation2d m_rightBackLocation = new Translation2d(12.75, -11);

  private final SwerveModule m_rightFront = new SwerveModule(RobotMap.rightFrontDrive, RobotMap.rightFrontTurn, RobotMap.rightFrontDriveEncoder, RobotMap.rightFrontTurnEncoder);
  private final SwerveModule m_leftFront = new SwerveModule(new CANSparkMax(8, MotorType.kBrushless), new WPI_VictorSPX(3), new AnalogEncoder(new AnalogInput(1)));
  private final SwerveModule m_leftBack = new SwerveModule(new CANSparkMax(9, MotorType.kBrushless), new WPI_VictorSPX(1), new AnalogEncoder(new AnalogInput(2)));
  private final SwerveModule m_rightBack = new SwerveModule(new CANSparkMax(10, MotorType.kBrushless), new WPI_VictorSPX(2), new AnalogEncoder(new AnalogInput(3)));

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_rightFrontLocation, m_leftFrontLocation, m_leftBackLocation, m_rightBackLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_rightFront.setDesiredState(swerveModuleStates[0]);
    m_leftFront.setDesiredState(swerveModuleStates[1]);
    m_leftBack.setDesiredState(swerveModuleStates[2]);
    m_rightBack.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_rightFront.getState(),
        m_leftFront.getState(),
        m_leftBack.getState(),
        m_rightBack.getState()
    );
  }
}
