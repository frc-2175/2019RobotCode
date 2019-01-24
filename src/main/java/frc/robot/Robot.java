/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.ServiceLocator;
import frc.command.Command;
import frc.info.RobotInfo;
import frc.log.LogServer;
import frc.log.RobotLogger;
import frc.subsystem.DrivetrainSubsystem;
import frc.subsystem.HatchIntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * - FRC Team                                                                                                                                       
 222222222222222      1111111   77777777777777777777555555555555555555 
2:::::::::::::::22   1::::::1   7::::::::::::::::::75::::::::::::::::5 
2::::::222222:::::2 1:::::::1   7::::::::::::::::::75::::::::::::::::5 
2222222     2:::::2 111:::::1   777777777777:::::::75:::::555555555555 
            2:::::2    1::::1              7::::::7 5:::::5            
            2:::::2    1::::1             7::::::7  5:::::5            
         2222::::2     1::::1            7::::::7   5:::::5555555555   
    22222::::::22      1::::l           7::::::7    5:::::::::::::::5  
  22::::::::222        1::::l          7::::::7     555555555555:::::5 
 2:::::22222           1::::l         7::::::7                  5:::::5
2:::::2                1::::l        7::::::7                   5:::::5
2:::::2                1::::l       7::::::7        5555555     5:::::5
2:::::2       222222111::::::111   7::::::7         5::::::55555::::::5
2::::::2222222:::::21::::::::::1  7::::::7           55:::::::::::::55 
2::::::::::::::::::21::::::::::1 7::::::7              55:::::::::55   
2222222222222222222211111111111177777777                 555555555     
The Fighting Calculators
*/
public class Robot extends TimedRobot {
  private boolean hasAutoEnded;

  private Joystick leftJoystick;
  private Joystick rightJoystick;
  private Joystick gamepad;

  private DrivetrainSubsystem drivetrainSubsystem;
  private HatchIntakeSubsystem hatchIntakeSubsystem;
  private RobotLogger robotLogger;
  private LogServer logServer;

  //WPI Lib Functions

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    hasAutoEnded = false;

    ServiceLocator.clear();
    
    robotLogger = new RobotLogger();
    new RobotInfo();
    logServer = new LogServer();

    // drivetrainSubsystem = new DrivetrainSubsystem();
    // hatchIntakeSubsystem = new HatchIntakeSubsystem();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    gamepad = new Joystick(2);

    logServer.runServer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot program is disabled and ready.");
    robotLogger.flush();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   * 
   * <p>You will need to initialize whatever autonomous command is selected 
   * inside this method.
   */
  @Override
  public void autonomousInit() {
    hasAutoEnded = false;
  }
  
  /**
   * This function is called periodically during autonomous.
   * This is where the autonomous commands must be executed.
   * @see frc.robot.Robot#executeCommand(Command)
   */
  @Override
  public void autonomousPeriodic() {
    robotLogger.log();
  }
  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // if (gamepad.getRawButton(9)) {
    //   if (VisionSubsystem.isTarget()) {
    //     drivetrainSubsystem.arcadeDrive(0, -Math.signum(VisionSubsystem.getTx()) / 5);
    //   }
    // } else {
    //   drivetrainSubsystem.blendedDrive(leftJoystick.getY(), -rightJoystick.getX());
    // }
    // if (leftJoystick.getRawButton(1)) {
    //   hatchIntakeSubsystem.spinIn();
    // } else if (rightJoystick.getRawButton(1)) {
    //   hatchIntakeSubsystem.spinOut();
    // } else {
    //   hatchIntakeSubsystem.stopSpinning();
    // }

    robotLogger.log();
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    robotLogger.log();
  }

  // Custom Functions

  /**
   * Runs the execute portion of a command until it is finished.
   * When it is finished, it'll call the end portion of the command
   * once.
   * <p>Note: this method will not call the initialize portion of the 
   * command.
   * @param command the command to execute
   */
  public void executeCommand(Command command) {
    if(!hasAutoEnded) {
      if(!command.isFinished()) {
        command.execute();
      } else {
        command.end();
        hasAutoEnded = true;
      }
    }
  }

  /**
   * Applies deadband onto input value
   * @param value input for value
   * @param deadband threshold for deadband
   * @return value with deadband
   */
  public static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}
}
