# C2025-Public
Code for Team 1678's 2025 robot, SubLime.

![Robot Image](/images/comp.png)

See also:
- Our CAD Release and Scouting Whitepaper
- Code from [2023](https://github.com/frc1678/C2023-Public/) and [2024](https://github.com/frc1678/C2024-Public/).

## Highlights
- LEDs
  
    Depending on the state that the robot is in, the LEDs are constantly updating to avoid having to look away from the robot to know what state it's in. Examples of these states include different colors for when in 'algae' versus 'coral' mode, and rapid blinks whenever a gamepiece is acquired or scored. For more information, you can look [at our LED subsystem](/src/main/java/frc/robot/subsystems/LEDs/LEDs.java) for reference.

- Controls

    SubLime only requires a single controller to utilize all of its functionality--the second controller serves as a debug controller incase something goes awry, and also allows for fine control for subsystems like the Climber when we need to zero.
  
    Our controlboard layout, [located here](/src/main/java/frc/robot/controlboard/ControlBoard.java), allows for scoring to happen both automatically (where the driver presses a single button to initiate the prep, drive, and score for coral, or a single button for the prep and drive along with a second button to score for algae), or for scoring to be done manually (where the driver does the same sequence as the automatic one plus an override button, which just prevents the drive from happening).

    SubLime's design means that it should be able to see an AprilTag from every coral scoring location on the reef--if no tag is seen, the driver is alerted so that they can either realign, or manually override to score.

- Detection & Vision

    SubLime has one Limelight 4 resting atop the bellypan, under the End Effector's resting position, which is always used for AprilTag detection and pose estimation. This camera is especially useful when we're aligning close to the reef for an auto score.

    The other Limelight 4 runs the Halio Accelerated Neural Network during auto (which is used for coral detection and coral pose estimation for ground pickup autos), and switches to an AprilTag pipeline during tele (which is used to improve accuracy on our net align).

## Notable Package Functions
- [`frc.robot.autos`](/src/main/java/frc/robot/autos/)

  Contains the base files along with specific auto routines grouped by type. For detection routines, we define specific zones on the field that a coral can be driven to in the auto, which allows us to filter out the marks. For net routines, we utilize separate net scoring actions atop the paths in order to ensure that the routine can score 3 algae + 1 coral in 15 seconds.

- [`frc.robot.subsystems`](/src/main/java/frc/robot/subsystems/)

  Contains all subsystems, including servos (position control subsystems, which are the Algae Deploy, Climber, Coral Deploy, Elevator, and Pivot), and motor subsystems (voltage control subsystems, which are the Algae Rollers, Climber Rollers, Coral Indexer, Coral Rollers, and End Effector), among others.

- [`frc.robot.subsystems.superstructure`](/src/main/java/frc/robot/subsystems/superstructure/)

  Holds all inter-subsystem sequencing (e.g. all commands to go to different scoring positions), most of our auto-drives, and our Motion Planner.

- [`frc.lib.drive`](/src/main/java/frc/lib/drive/)

  Holds all the drive-related commands we use for auto aligning and auto scoring.
  
- [`frc.lib.sim`](/src/main/java/frc/lib/sim/)

  Holds all simulation-related files. 

- [`frc.lib.io`](/src/main/java/frc/lib/io/)

  Holds all IO files used for our subsystems.

  

