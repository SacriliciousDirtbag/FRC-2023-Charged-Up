package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

//import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.State.FState;
import frc.robot.State.HState;
import frc.robot.State.VState;
import frc.robot.autos.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0); //Logitech XboxController
    private final Joystick driver2 = new Joystick(0); //Logitech Extreme3D Pro
    public final Joystick buttonBoard = new Joystick(1);//External Driver
    public final Joystick buttonBoard2 = new Joystick(2); //External Driver 2


    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int translationAxis = Joystick.kDefaultYChannel;
    private final int strafeAxis = Joystick.kDefaultXChannel;
    private final int rotationAxis = Joystick.kDefaultZChannel;

    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    /* 
    
    // LOGITECH - XBOX CONTROLLER //

    //DRIVE
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    //INTAKE
    private final JoystickButton accumulatorOut = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //TODO: integrate button mapping
    //private final int accumulatorIn = XboxController.Axis.kLeftTrigger.value; //TODO: integrate button mapping
    private final JoystickButton accumulatorIn = new JoystickButton(driver, XboxController.Button.kX.value);

    //WHEELS
    private final JoystickButton wheelForward = new JoystickButton(driver, XboxController.Button.kStart.value); //TODO: integrate button mapping
    //private final int accumulatorIn = XboxController.Axis.kLeftTrigger.value; //TODO: integrate button mapping
    private final JoystickButton wheelReverse = new JoystickButton(driver, XboxController.Button.kBack.value);

    //ELEVATOR
    private final JoystickButton elevatorUpButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value); //TODO: integrate button mapping
    //private final int elevatorDownButton = XboxController.Axis.kRightTrigger.value; //TODO: integrate button mapping
    private final JoystickButton elevatorDownButton = new JoystickButton(driver, XboxController.Button.kB.value);

    */


    // LOGITECH PRO - FLIGHTSTICK //

    //DRIVE
    
    private final JoystickButton zeroGyro = new JoystickButton(driver2, 11); //TODO: Implement As Button
    private final JoystickButton robotCentric = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
    /*
    //INTAKE
    private final JoystickButton accumulatorOut = new JoystickButton(driver2, 5);
    private final JoystickButton accumulatorIn = new JoystickButton(driver2, 3);

    //INTAKE
    private final JoystickButton flipOut = new JoystickButton(driver2, 7);
    private final JoystickButton flipIn = new JoystickButton(driver2, 8);

    //WHEELS
    private final JoystickButton wheelForward = new JoystickButton(driver2, 2);
    private final JoystickButton wheelReverse = new JoystickButton(driver2, 1);

    //ELEVATOR
    private final JoystickButton elevatorUpButton = new JoystickButton(driver2, 6);
    private final JoystickButton elevatorDownButton = new JoystickButton(driver2, 4);
    */
    
    private final JoystickButton safeButton = new JoystickButton(driver2, 12); //Reset Button - use later for Assist Controller
    private final JoystickButton safeButton2 = new JoystickButton(driver2, 12); //Reset Button

    // BUTTON BOARD //

    private final JoystickButton TOP_ORANGE = new JoystickButton(buttonBoard, 1); //TOP ORANGE
    private final JoystickButton BOTTOM_ORANGE = new JoystickButton(buttonBoard, 3); //BOTTOM ORANGE
    private final JoystickButton TOP_RED = new JoystickButton(buttonBoard, 2); //TOP RED
    private final JoystickButton BOTTOM_RED = new JoystickButton(buttonBoard, 4); //BTTOM RED
    private final JoystickButton TOP_BLUE = new JoystickButton(buttonBoard, 10); //TOP BLUE
    private final JoystickButton MIDDLE_BLUE = new JoystickButton(buttonBoard, 5);
    private final JoystickButton BOTTOM_BLUE = new JoystickButton(buttonBoard, 6); //BOTTOM BLUE
    private final JoystickButton TOP_GREEN = new JoystickButton(buttonBoard, 7); //TOP GREEN
    private final JoystickButton BOTTOM_GREEN = new JoystickButton(buttonBoard, 9); //BOTTOM GREEN
    
    /*
    //INTAKE
    private final JoystickButton accumulatorOut = new JoystickButton(buttonBoard2, 5);
    private final JoystickButton accumulatorIn = new JoystickButton(buttonBoard2, 3);

    //INTAKE
    private final JoystickButton flipOut = new JoystickButton(buttonBoard2, 7);
    private final JoystickButton flipIn = new JoystickButton(buttonBoard2, 8);

    //WHEELS
    private final JoystickButton wheelForward = new JoystickButton(buttonBoard2, 2);
    private final JoystickButton wheelReverse = new JoystickButton(buttonBoard2, 1);

    //ELEVATOR
    private final JoystickButton elevatorUpButton = new JoystickButton(buttonBoard2, 6);
    private final JoystickButton elevatorDownButton = new JoystickButton(buttonBoard2, 4);
    */

    /* Subsystems & Commands */
    public final Swerve s_Swerve = new Swerve();
    public final ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
    public final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
    public final LEDSubsystem s_lightSubsystem = new LEDSubsystem();

    public final Command m_leftCommand = new left(s_Swerve);
    public final Command m_middleCommand = new middle(s_Swerve);
    public final Command m_rightCommand = new right(s_Swerve);
    public final Command s_HighCubeAuto = new SequentialCommandGroup(
        new HighCubeAuto(s_IntakeSubsystem, s_ElevatorSubsystem),
        m_middleCommand
        //new RunCommand(() -> s_Swerve.drive(new Translation2d(-1, 0), 0, true, true), s_Swerve)
    );
    
    public final Command s_AutoBalance = new NewAutoBalance(s_Swerve);

    public final Command s_HighCubeOnly = new SequentialCommandGroup(
        new HighCubeAuto(s_IntakeSubsystem, s_ElevatorSubsystem)
    );

    public final IntakeCone s_IntakeCone = new IntakeCone(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final IntakeCube s_IntakeCube = new IntakeCube(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final RampCone s_RampCone = new RampCone(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final RampCube s_RampCube = new RampCube(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final ScoreConeHigh s_ScoreConeHigh = new ScoreConeHigh(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final ScoreConeLow s_ScoreConeLow = new ScoreConeLow(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final ScoreCubeHigh s_ScoreCubeHigh = new ScoreCubeHigh(s_ElevatorSubsystem, s_IntakeSubsystem);
    public final ScoreCubeLow s_ScoreCubeLow = new ScoreCubeLow(s_ElevatorSubsystem, s_IntakeSubsystem);

    public final IntakeReset s_IntakeReset = new IntakeReset(s_ElevatorSubsystem, s_IntakeSubsystem);

    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        // Configure the button bindings
        configureButtonBindings();

        m_Chooser.addOption("Middle Auto", m_middleCommand);
        m_Chooser.addOption("Left Auto", m_leftCommand);
        m_Chooser.addOption("Right Auto", m_rightCommand);
        m_Chooser.setDefaultOption("High Cube Auto", s_HighCubeAuto);
        m_Chooser.setDefaultOption("High Plan B", s_HighCubeOnly);
        m_Chooser.setDefaultOption("New Auto Balance", s_AutoBalance);
        SmartDashboard.putData(m_Chooser);
    }
        
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        //M_ELEVATOR_EXTEND_BUTTON = ()-> driver.getRawButton(3);
        //M_ELEVATOR_RETRACT_BUTTON = ()-> driver.getRawButton(2);

        //accumulatorIn.onTrue(new InstantCommand(() -> CentralCommand.getAsBooleanFalse()));
        //accumulatorOut.onTrue(new InstantCommand(() -> CentralCommand.getAsBooleanTrue()));
        
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

       //elevatorUpButton.onTrue(new InstantCommand(() -> s_ElevatorSubsystem.raiseArm()));

        /* POSES */
/* 
       //RAMP POSE
        TOP_ORANGE.onTrue(new InstantCommand(() -> s_RampCone.initialize()));
        //TOP_ORANGE.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopExtend()));

        BOTTOM_ORANGE.onTrue(new InstantCommand(() -> s_RampCube.initialize()));
        //BOTTOM_ORANGE.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopExtend()));

        //INTAKE POSE
        TOP_RED.onTrue(new InstantCommand(() -> s_IntakeCone.initialize())); 
        //TOP_RED.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopArm()));
        
        BOTTOM_RED.onTrue(new InstantCommand(() -> s_IntakeCube.initialize())); 
        //BOTTOM_RED.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopArm()));

        //SCORE CONE POSE
        TOP_BLUE.onTrue(new InstantCommand(() -> s_ScoreConeHigh.initialize())); 
        //TOP_BLUE.onFalse(new InstantCommand(() -> s_IntakeSubsystem.setTargetPosition()));
        
        BOTTOM_BLUE.onTrue(new InstantCommand(() -> s_ScoreConeLow.initialize()));
        //BOTTOM_BLUE.onFalse(new InstantCommand(() -> s_IntakeSubsystem.setTargetPosition()));


        //SCORE CUBE POSE
        TOP_GREEN.onTrue(new InstantCommand(() -> s_ScoreCubeHigh.initialize()));
        //TOP_GREEN.onFalse(new InstantCommand(() -> s_IntakeSubsystem.resetAll()));
        
        BOTTOM_GREEN.onTrue(new InstantCommand(() -> s_ScoreCubeLow.initialize()));
        //BOTTOM_GREEN.onFalse(new InstantCommand(() -> s_IntakeSubsystem.resetAll()));

*/
        /* DRIVER CONTROLS */


        //ELEVATOR - MANUAL
        TOP_ORANGE.onTrue(new InstantCommand(() -> s_ElevatorSubsystem.elevatorUp()));
        TOP_ORANGE.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopElevator()));

        BOTTOM_ORANGE.onTrue(new InstantCommand(() -> s_ElevatorSubsystem.elevatorDown()));
        BOTTOM_ORANGE.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.stopElevator()));

        //EXTEND, POSE
        /*TOP_RED.onTrue(new InstantCommand(() -> s_ScoreConeHigh.initialize())); 
        TOP_RED.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.carriageRetract()));
        
        BOTTOM_RED.onTrue(new InstantCommand(() -> s_ScoreConeLow.initialize()));
        BOTTOM_RED.onFalse(new InstantCommand(() -> s_ElevatorSubsystem.carriageRetract()));
        */
        //INTAKE POSES
        TOP_BLUE.onTrue(new InstantCommand(() -> s_RampCube.initialize())); 
        TOP_BLUE.onFalse(new InstantCommand(() -> s_IntakeReset.initialize()));

        MIDDLE_BLUE.onTrue(new InstantCommand(() -> s_ScoreCubeHigh.initialize())); 
        MIDDLE_BLUE.onFalse(new InstantCommand(() -> s_IntakeReset.initialize()));
        
        BOTTOM_BLUE.onTrue(new InstantCommand(() -> s_IntakeCube.initialize()));
        BOTTOM_BLUE.onFalse(new InstantCommand(() -> s_IntakeReset.initialize()));

        //WHEEL SPIN - MANUAL
        TOP_GREEN.onTrue(new InstantCommand(() -> s_IntakeSubsystem.wheelForward()));
        TOP_GREEN.onFalse(new InstantCommand(() -> s_IntakeSubsystem.resetAll()));
        
        BOTTOM_GREEN.onTrue(new InstantCommand(() -> s_IntakeSubsystem.wheelReverse()));
        BOTTOM_GREEN.onFalse(new InstantCommand(() -> s_IntakeSubsystem.resetAll()));

        //MISC
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); //11
        safeButton.onTrue(new InstantCommand(() -> s_ElevatorSubsystem.resetAll())); //10
        //safeButton2.onTrue(new InstantCommand(() -> s_ElevatorSubsystem.resetAll())); //12


        //LEDS
        /*button1.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest()));
        button2.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest2()));
        button3.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest3()));
        button4.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest4()));
        button5.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest5()));
        button6.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest6()));
        button7.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest7()));
        button8.onTrue(new InstantCommand(()-> s_lightSubsystem.lightTest8()));*/
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     public Command getAutonomousCommand(){
        return m_Chooser.getSelected();
     }

    public Command getBalanceCommand() {
        // An ExampleCommand will run in autonomous
        return Commands.sequence(
            new RunCommand(() -> s_Swerve.drive(new Translation2d(-2, 0), 0, true, true), s_Swerve).withTimeout(1),
            new NewAutoBalance(s_Swerve)
        );
    }

    public Command getTurnCommand(){
        return new autoTurn();
    }

    public Command getShooterCommand(){
        return new shooter();
    }
}