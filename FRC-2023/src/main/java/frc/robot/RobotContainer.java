
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import com.ctre.phoenix.music.Orchestra;
import com.fasterxml.jackson.databind.PropertyNamingStrategies.SnakeCaseStrategy;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.AutoSequence;
import frc.robot.commands.Autonomous.CubeAuto;
import frc.robot.commands.Autonomous.ReleaseArm;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.DriveForward;
import frc.robot.commands.Drivetrain.DriveForwardPID;
import frc.robot.commands.Drivetrain.ForwardForTime;
import frc.robot.commands.Drivetrain.SetIdle;
// import frc.robot.commands.Elevator.EncoderTest;
import frc.robot.commands.Elevator.JoystickElevator;
import frc.robot.commands.Elevator.RecalibrateElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevator;
import frc.robot.commands.Elevator.ElevatorExtensionModes.ExtendElevatorSmart;
import frc.robot.commands.Elevator.Feedforward.ExtendElevatorSmartWithFeedforward;
//import frc.robot.commands.Gyro.GyroBalance;
import frc.robot.commands.IntakeArm.ArmDown;
import frc.robot.commands.IntakeArm.GoToAngle;
import frc.robot.commands.IntakeArm.GoToAngleSmart;
import frc.robot.commands.IntakeArm.JoystickArm;
import frc.robot.commands.IntakeArm.RecalibrateArm;
import frc.robot.commands.IntakeArm.Feedforward.GoToAngleSmartWithFeedForward;
import frc.robot.commands.Limelight.AutoTrackPole;
import frc.robot.commands.Limelight.TestServo;
import frc.robot.commands.Limelight.TrackTarget;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.LimelightTestTurret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final class Config{
    //Joysticks
    public static final int kDriverJoystickID = 0;
    public static final int kSecondJoystickID = 1;

    //Intake 
    public static final int kIntakeInID = 1;
    public static final int kIntakeOutID = 2;
    public static final int kCubeInID = 3;
    public static final int kCubeOutID = 4;

    //Elevator
    public static final int kRecalElevatorID = 8;
    public static final int kHalfElevatorID = 7;
    public static final int kTestExtendID = 7;

    //Arm
    public static final int kRecalibrateID = 4; //FOR BOTH ARM AND ELEVATOR
    public static final int kArmDownID = 3; //NOTE: Conflicts with TestArm, do not use together
    public static final int kHalfArmID = 3;
    public static final int kTestArmID = 3;

    //Scoring
    public static final int kLowScoreID = 5;
    public static final int kHighScoreID = 6;

    //Auto
    public static final double kTimeInSecsShort = 5;
    public static final double kTimeInSecsLong = 6.75;


    public static final String kMoveBack = "move_back_auto";
    public static final String kReleaseArmAuto = "release_arm_auto";
    public static final String kCubeAuto = "cube_auto";
    public static final String kSimpleCubeAuto = "simple_cube_auto";

    /* Driver Controls:
     *  - Left Joystick = Speed
     *  - Right Joystick = Turn
     * 
     * Secondary Controls:
     *  - A = Intake In (release for off)
     *  - B = Intake Out (release for off)
     *  - Y = Recalibrate
     *  - X = ArmDown
     *  - Left Bumper = Low Score
     *  - Right Bumper = High Score
     */

    public static final int kArmUpButtonID = 0;
    public static final int kArmDownButtonID = 3;
    public static final double kArmSetpoint = 7;
    
    public static final int kForwardJoystickButtonID = 5;
    public static final int kForwardForTimeButtonID = 0; //placeholder
    public static final double kTime = 0;
  }

  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  //Joysticks
  private Joystick m_driveJoystick = new Joystick(Config.kDriverJoystickID); 
  private Joystick m_secondJoystick = new Joystick(Config.kSecondJoystickID);

  // //Drivetrain (on Driver Controller)
  private Drivetrain m_drivetrain = new Drivetrain();
  private ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drivetrain, m_driveJoystick);
  private SetIdle m_setIdle = new SetIdle(m_drivetrain);

  // //On Secondary controller
  //Intake
  private RollerIntake m_rollerIntake = new RollerIntake();
  private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, Config.kIntakeInID);
  private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, Config.kIntakeOutID);
  private JoystickButton m_cubeInButton = new JoystickButton(m_secondJoystick, Config.kCubeInID);
  private JoystickButton m_cubeOutButton = new JoystickButton(m_secondJoystick, Config.kCubeOutID);
  
  //Elevator
  private Elevator m_elevator = new Elevator();
  private JoystickElevator m_joystickElevator = new JoystickElevator(m_elevator, m_secondJoystick); //Change to second joystick later

  private RecalibrateElevator m_recalibrateElevator = new RecalibrateElevator(m_elevator);
  private JoystickButton m_recalibrateElevButton = new JoystickButton(m_secondJoystick, Config.kRecalElevatorID);

  //Elevator Extensions: Pass in encoder ticks for specific angle
  private ExtendElevator m_halfExtensionElevator = new ExtendElevator(m_elevator, m_elevator.getEncoderLimitUp()/2);
  private JoystickButton m_halfExtendButton = new JoystickButton(m_secondJoystick, Config.kHalfElevatorID);
  private ExtendElevatorSmart m_lowScoreElevator = new ExtendElevatorSmart(m_elevator, -61);
  private ExtendElevatorSmart m_highScoreElevator = new ExtendElevatorSmart(m_elevator, -61);

  private ExtendElevatorSmart m_testExtendElevator = new ExtendElevatorSmart(m_elevator, m_elevator.getEncoderLimitUp() * 0.75);
  private JoystickButton m_testExtendButton = new JoystickButton(m_secondJoystick, Config.kTestExtendID);
 
  //Arm
  private Arm m_arm = new Arm();
  private JoystickArm m_joystickArm = new JoystickArm(m_arm, m_secondJoystick);

  private RecalibrateArm m_recalibrateArm = new RecalibrateArm(m_arm);
  private ArmDown m_armDown = new ArmDown(m_arm);
  private JoystickButton m_armDownButton = new JoystickButton(m_secondJoystick, Config.kArmDownID);

  private GoToAngle m_halfAngleArm = new GoToAngle(m_arm, m_arm.getEncoderLimitUp()/2);
  private JoystickButton m_halfAngleButton = new JoystickButton(m_secondJoystick, Config.kHalfArmID);

  private GoToAngleSmart m_lowScoreArm = new GoToAngleSmart(m_arm, 52);
  private GoToAngleSmart m_highScoreArm = new GoToAngleSmart(m_arm, 62);
  
  private GoToAngleSmart m_testAngleArm = new GoToAngleSmart(m_arm, m_arm.getEncoderLimitUp() * 0.5);
  private JoystickButton m_testAngleButton = new JoystickButton(m_secondJoystick, Config.kTestArmID);
  
  private ExtendElevatorSmartWithFeedforward m_lowElevatorFeedforward = new ExtendElevatorSmartWithFeedforward(m_elevator, 61);
  private ExtendElevatorSmartWithFeedforward m_highElevatorFeedforward = new ExtendElevatorSmartWithFeedforward(m_elevator,  61);
  private GoToAngleSmartWithFeedForward m_lowArmFeedforward = new GoToAngleSmartWithFeedForward(m_arm, 52);
  private GoToAngleSmartWithFeedForward m_highArmFeedforward = new GoToAngleSmartWithFeedForward(m_arm, 62);

  //Elevator + Arm Scoring Parallel Groups
  private ParallelCommandGroup m_lowScore = new ParallelCommandGroup(m_lowElevatorFeedforward, m_lowArmFeedforward);
  private ParallelCommandGroup m_highScore = new ParallelCommandGroup(m_highElevatorFeedforward, m_highArmFeedforward); 

  private JoystickButton m_lowScoreButton = new JoystickButton(m_secondJoystick, Config.kLowScoreID);
  private JoystickButton m_highScoreButton = new JoystickButton(m_secondJoystick, Config.kHighScoreID);

  //recalibration
  private JoystickButton m_recalibrateButton = new JoystickButton(m_secondJoystick, Config.kRecalibrateID);

  /* Working arm/elevator combos:
   * Ground: Elevator = , Arm =
   * Cone Low: Elevator = -61, Arm = 52
   * Cone High: Elevator = , Arm = 
   * Cube Low: Elevator = , Arm = 
   * Cube High: Elevator = , Arm = 
   * Shelf: Elevator = , Arm = 
   */

  //Auto
  private ReleaseArm m_releaseArm = new ReleaseArm(m_arm);
  private AutoSequence m_autoSequenceShort = new AutoSequence(m_drivetrain, m_arm, Config.kTimeInSecsShort);
  private AutoSequence m_autoSequenceLong = new AutoSequence(m_drivetrain, m_arm, Config.kTimeInSecsLong);

  private CubeAuto m_cubeAutoShort = new CubeAuto(m_arm, m_rollerIntake, m_drivetrain, Config.kTimeInSecsShort);
  private CubeAuto m_cubeAutoLong = new CubeAuto(m_arm, m_rollerIntake, m_drivetrain, Config.kTimeInSecsLong);
  private CubeAuto m_simpleCubeAuto = new CubeAuto(m_arm, m_rollerIntake, m_drivetrain, 0);

  // private Limelight m_limelight = new Limelight();
  // private Turret m_turret = new Turret();
  // private TrackTarget m_track = new TrackTarget(m_limelight, m_turret);
  // private TestServo m_test = new TestServo(m_turret);
  // Joystick buttons
  // private JoystickButton m_forwardButton = new JoystickButton(m_driveJoystick, Config.kForwardJoystickButtonID);
  // private DriveForward m_driveForward = new DriveForward(m_drivetrain, m_forwardButton);

  private SendableChooser<Boolean> m_pipeChooser = new SendableChooser<>();

  // private Limelight m_limelight = new Limelight();
  // private Turret m_turret = new Turret();
  // private TrackTarget m_track = new TrackTarget(m_limelight, m_turret);
  // private TestServo m_test = new TestServo(m_turret);
  // Joystick buttons
  // private JoystickButton m_forwardButton = new JoystickButton(m_driveJoystick, Config.kForwardJoystickButtonID);
  // private DriveForward m_driveForward = new DriveForward(m_drivetrain, m_forwardButton);

  // // private ForwardForTime m_ForwardForTime = new ForwardForTime(m_drivetrain,  3);
  // // private JoystickButton m_ForwardForTimeButton = new JoystickButton(m_driveJoystick, Config.kForwardForTimeButtonID);

  // // private DriveForwardPID m_drivePID = new DriveForwardPID(m_drivetrain);

  // private GyroBalance m_balance = new GyroBalance(m_drivetrain);

 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_autoChooser.setDefaultOption(Config.kReleaseArmAuto, m_releaseArm);
    // m_autoChooser.addOption(Config.kMoveBack + "_short", m_autoSequenceShort);
    // m_autoChooser.addOption(Config.kCubeAuto + "_short", m_cubeAutoShort);
    // m_autoChooser.addOption(Config.kMoveBack + "_long", m_autoSequenceLong);
    // m_autoChooser.addOption(Config.kCubeAuto + "_long", m_cubeAutoLong);
    // m_autoChooser.addOption(Config.kSimpleCubeAuto, m_simpleCubeAuto);
    // SmartDashboard.putData(m_autoChooser);

    // m_pipeChooser.setDefaultOption("Reflective Tape", m_limelight.setPipeline(1));
    // m_pipeChooser.addOption("Cone", m_limelight.setPipeline(2));
    // m_pipeChooser.addOption("Cube", m_limelight.setPipeline(3));
    // SmartDashboard.putData(m_pipeChooser);
    // // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Intake in
    m_intakeInButton.onTrue(m_rollerIntake.turnOnIntake());
    m_intakeOutButton.onTrue(m_rollerIntake.turnEjectIntake());
    // m_cubeInButton.onTrue(m_rollerIntake.intakeCube());
    // m_cubeOutButton.onTrue(m_rollerIntake.ejectCube());

    //Intake out
    m_intakeInButton.onFalse(m_rollerIntake.turnOffIntake());
    m_intakeOutButton.onFalse(m_rollerIntake.turnOffIntake());
    // m_cubeInButton.onFalse(m_rollerIntake.turnOffCube());
    // m_cubeOutButton.onFalse(m_rollerIntake.turnOffCube());

    //Elevator control
    m_recalibrateElevButton.onTrue(m_recalibrateElevator);
    m_halfExtendButton.onTrue(m_halfExtensionElevator);
    m_testExtendButton.onTrue(m_testExtendElevator);

    //Arm control
    m_recalibrateButton.onTrue(m_recalibrateArm);
    m_recalibrateButton.onTrue(m_recalibrateElevator);
    // m_halfAngleButton.onTrue(m_halfAngleArm);
    // m_testAngleButton.onTrue(m_testAngleArm);
    m_armDownButton.onTrue(m_armDown);

    //Score
    m_lowScoreButton.onTrue(m_lowScore);
    m_highScoreButton.onTrue(m_highScore);

    // m_ForwardForTimeButton.onTrue(m_ForwardForTime);
    // m_forwardButton.onTrue(m_driveForward);
    // m_ArmUpButton.onTrue(m_goToAngle);
    // m_ArmDownButton.onTrue(m_armDown);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
 public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public Command getTeleopCommand(){
    // m_drivetrain.setDefaultCommand(m_balance);
    // m_drivetrain.setDefaultCommand(m_arcadeDrive);
    m_drivetrain.setDefaultCommand(m_setIdle);
    m_elevator.setDefaultCommand(m_lowScore);
    m_arm.setDefaultCommand(m_lowScore);
    // m_arm.setDefaultCommand(m_joystickArm);

    // m_limelight.setDefaultCommand(m_track);
    // m_turret.setDefaultCommand(m_test);
    // m_track.schedule();

    // m_encoderTest.schedule(); 
    return null;
  }
}
