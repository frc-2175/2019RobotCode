/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.command.Command;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
/*                                                                                                                                         
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
*/
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean hasAutoEnded;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    hasAutoEnded = false;
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
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    hasAutoEnded = false;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
