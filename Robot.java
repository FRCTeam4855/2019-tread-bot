/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The ccompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  
  float Right, Left;
  float desiredHeading = 0;
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  Victor leftmotor = new Victor(0);
  // This defines the left  motor 
   Victor rightmotor = new Victor(1);
  // This defines the right motor

  // define the differential drive
  DifferentialDrive drive = new DifferentialDrive(leftmotor, rightmotor);


  Joystick Controller = new Joystick(0);
  // This defines the Controller
  double LY, RY, LT, RT;
  // This defines the Left joystick, Right Joystick, Left Trigger, and Right Trigger, as a decimal. 
  boolean B;
  // This defines the B button as a true or falsehto
  
  /*
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

 
  @Override
  public void teleopPeriodic() {
  LY = Controller.getRawAxis(1);
  // This is for the Left Joystick
  RY = Controller.getRawAxis(5);
  // This is for the Right Joystick 

  //leftmotor.set(LY);
  //rightmotor.set(RY);

  
  if (LY <= .1 && LY >= -.1) { 
    // This is for a deadzone so the battery will not go dead faster than it should.
    LY = 0.0;
  }
  if (RY <= .1 && RY >= -.1) { 
     //This is for a deadzone so the battery will not go dead faster than it should.
    RY = 0.0;

  }
  drive.tankDrive(-LY - .3, -RY - .3);

  }
  
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // autonomous goodies


  @Override
public void autonomousInit() {
  super.autonomousInit();
  desiredHeading = 0;
  ahrs.reset();
// This makes it that it will always try to go to 0 degrees, or the front of then navX
  Command c = new AutonomousCommand();
  c.start();
}

@Override
public void autonomousPeriodic() {
  super.autonomousPeriodic();

  // we need this so commands run!
  Scheduler.getInstance().run();
}
 
/**
 * normal an angle to the range -180..180
 * @param angle
 * @return
 */
double normalizeAngle (double angle) {
  while (angle > 180){
    angle = angle - 360;
  }
  while (angle < -180){
    angle = angle + 360;
  }
  return angle;
}

class EmptyCommand extends Command {
  @Override
  protected void initialize() {
    super.initialize();
  }
  @Override
  protected void execute() {
    super.execute();
  }
  @Override
  protected boolean isFinished() {
    return false;
  }
  @Override
  protected void end() {
    super.end();
  }
  @Override
  protected void interrupted() {
    super.interrupted();
  }

}

class AutonomousCommand extends CommandGroup {
  AutonomousCommand() {
    addSequential(new GoStraightCommand(3));
    addSequential(new TurnCommand(90));
    addSequential(new GoStraightCommand(3));
    addSequential(new TurnCommand(-90));
    addSequential(new GoStraightCommand(3));
    addSequential(new TurnCommand(180));
    addSequential(new GoStraightCommand(6));
    addSequential(new TurnCommand(90));
    addSequential(new GoStraightCommand(3));
    addSequential(new TurnCommand(90));
    addSequential(new GoStraightCommand(1));
  }
  //This is the code, which actaully makes it move, the duration is three seconds. 
}


class TurnCommand extends Command {
  float amountToTurn; 
  float whichWayToTurn;
  boolean weAreDone;
  Timer timer = new Timer();
  TurnCommand(float _amountToTurn) {
    amountToTurn = _amountToTurn;

  }
  @Override
  protected void initialize() {
    float currentHeading = ahrs.getYaw();
    desiredHeading = desiredHeading + amountToTurn;
    double error = (currentHeading - desiredHeading);
    // Positive error means we are too far to the right
    error = normalizeAngle(error);
    if(error  > 0 )  {
      whichWayToTurn = -1;
    }
    else {
      whichWayToTurn = 1;
    }
  System.out.println("Turn command started" + this + ", gonna turn " + amountToTurn + ", new heading = " + desiredHeading);
    timer.reset();
    timer.start();
  }
  @Override
  protected void execute() {
    float currentHeading = ahrs.getYaw();
    SmartDashboard.putNumber("Angle", currentHeading);
  // This makes it that it shows the angle from the navX, left is negative, right is positive
    double error = (currentHeading - desiredHeading);
    error = normalizeAngle(error);
  
    SmartDashboard.putNumber("Error", error);
  
    double curve = error / -60;
    if (0 < curve && curve < .75){
      curve = .75;
    }
    if (-.75 < curve && curve < 0){
      curve = -.75;
    }
    SmartDashboard.putNumber("Curve", curve);
    System.out.println("heading = " + currentHeading + ", error " + error + ", turning " + curve);
  
    // false turns off squaring
    drive.arcadeDrive(0, curve, false);

    weAreDone = false;
    if (whichWayToTurn > 0) {
      //We are turning right
      if (error >= 0) {
        //If we overshot, we are done
        weAreDone = true;
      }
    } else {
      //We are turning left
      if (error <= 0) {
        //If we overshot, we are done
        weAreDone = true;
      }
    }

    // finish the command in 2 seconds, even if we are not on the
    // correct heading
    if (timer.get() > 2.0) {
      //weAreDone = true;
    }
  }
  @Override
  protected boolean isFinished() {
    return weAreDone;
  }
  @Override
  protected void end() {
    System.out.println("turn command done$" + this);
    drive.stopMotor();
  }
  @Override
  protected void interrupted() {
    drive.stopMotor();
  }

}


class GoStraightCommand extends Command {
  Timer timer = new Timer();
  float Distance;
  GoStraightCommand(float _Distance) {
    Distance = _Distance;
  }
  @Override
  protected void initialize() {
    timer.reset();
    timer.start();
    System.out.println("GoStraightCommand started" + this);
  }
  @Override
  protected void execute() {
    float currentHeading = ahrs.getYaw();
    SmartDashboard.putNumber("Angle", currentHeading);
  
    double error = (currentHeading - desiredHeading);
    error = normalizeAngle(error);
  
    SmartDashboard.putNumber("Error", error);
  
    double curve = error / -30;
    SmartDashboard.putNumber("Curve", curve);
  
    // false turns off squaring
    drive.arcadeDrive(.5, curve, false);
  }
  @Override
  protected boolean isFinished() {
    if (timer.get() > Distance) {
      return true;
    }
    return false;
  }
  @Override
  protected void end() {
    System.out.println("GoStraightCommand ended" + this);
    drive.stopMotor();
  }
  @Override
  protected void interrupted() {
    drive.stopMotor();
  }

}

}
