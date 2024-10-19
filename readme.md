# Disclaimer:

This is an experimental project. Things here may change at any time. If your robot explodes because of a change here, we give no warrenty.

This project is licensed BSD-2-Clause.

# ReduxFTC: FTC libraries for Redux products

This repository provides software libraries for Redux products compatible and intended to be used 
with the FTC control system.

Code in here may or may not eventually get upstreamed into the official SDK; however, in the 
meantime, this library exists to provide the .aars necessary for Blocks/OnBot support.

## Documentation:
 - [Canandgyro Overview](https://docs.reduxrobotics/canandgyro)
 - [Javadocs](https://apidocs.reduxrobotics.com/current/ftcjava)
 - [Example programs](https://github.com/Redux-Robotics/ReduxFTC/tree/main/examples/src/main/java/org/firstinspires/ftc/teamcode)
 
## Installation instructions:

**This requires SDK v10.0+**. If you are using an older SDK version, please update!

### Installation for OnBot/Blocks:

1. Download the latest reduxftc.aar here.
2. Navigate to the Control Hub web interface at http://192.168.43.1:8080 and navigate to 
   **OnBotJava**. 
3. Select the **Upload Files** button in the top-left. 
4. Select the downloaded reduxftc.aar to upload.
5. When the file is done uploading, close the Uploading Files dialog.
   Then, **restart the Robot Controller!!!!** This can be accomplished most easily by
   **turning the robot off and then on again.**
6. The library should be installed now. You can verify installation worked by checking to see if
   devices such as "Redux Canandgyro (Analog)" appear as an option for the analog ports in the
   robot configuration menu. 

### Installation for Android Studio:

In your `build.dependencies.gradle` in the root of your FTCRobotController project add the Redux 
Maven repository and the `implementation` line:

```groovy
repositories {
    // ...
    // other maven repositories, don't touch the rest
    // ...
   
   // Add the below line in the repositories block vvv
    maven { url = "https://maven.reduxrobotics.com/" }
   // Add the above line in the repositories block ^^^
   
}

dependencies {
   
   // Add the below line in the dependencies block vvv
   implementation 'org.reduxrobotics.ftc:reduxftc:2024.1.0'
   // Add the above line in the dependencies block ^^^
   
   // ...
   // other dependencies, don't touch the rest
   // ...
}
```

Run a gradle sync and you should be able to use the library.

## Changelog:

### v2024.1.0
 - Initial release
