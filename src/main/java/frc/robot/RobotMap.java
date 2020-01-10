
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code
*/
/* must be accompanied by the FIRST BSD license file in the root directory of
*/
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // ShoppingKart Drive
    public static SpeedController rightFrontDrive = new CANSparkMax(7, MotorType.kBrushless);
    public static SpeedController rightFrontTurn = new WPI_VictorSPX(5);
    public static CANEncoder rightFrontDriveEncoder = ((CANSparkMax) rightFrontDrive).getEncoder();
    public static AnalogEncoder rightFrontTurnEncoder = new AnalogEncoder(new AnalogInput(0));

    public static SpeedController leftFrontDrive = new CANSparkMax(8, MotorType.kBrushless);
    public static SpeedController leftFrontTurn = new WPI_VictorSPX(3);
    public static CANEncoder leftFrontDriveEncoder = ((CANSparkMax) leftFrontDrive).getEncoder();
    public static AnalogEncoder leftFrontTurnEncoder = new AnalogEncoder(new AnalogInput(1));

    public static SpeedController leftBackDrive = new CANSparkMax(9, MotorType.kBrushless);
    public static SpeedController leftBackTurn = new WPI_VictorSPX(1);
    public static CANEncoder leftBackDriveEncoder = ((CANSparkMax) leftBackDrive).getEncoder();
    public static AnalogEncoder leftBackTurnEncoder = new AnalogEncoder(new AnalogInput(2));

    public static SpeedController rightBackDrive = new CANSparkMax(10, MotorType.kBrushless);
    public static SpeedController rightBackTurn = new WPI_VictorSPX(2);
    public static CANEncoder rightBackDriveEncoder = ((CANSparkMax) rightBackDrive).getEncoder();
    public static AnalogEncoder rightBackTurnEncoder = new AnalogEncoder(new AnalogInput(3));

    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

}