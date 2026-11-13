# 2026 REBUILT

FRC Team 8016's 2026 Java robot project (WPILib / GradleRIO 2026.2.1).

## Code with students in GitHub Codespaces

1. Select the `add-2026-devcontainer` branch on GitHub, then choose **Code → Codespaces → Create codespace**. After this branch is merged, you can use the default branch instead.
2. Wait for the container build and the setup command, `./gradlew --no-daemon compileJava`, to finish before working with the Java language server. First setup needs an internet connection and may take several minutes.
3. If you already have a Codespace, pull the latest changes, open the command palette (**Ctrl+Shift+P**, or **Cmd+Shift+P** on macOS), and run **Codespaces: Rebuild Container** to install the desktop support.

The container includes Java 17, the matching WPILib 2026.2.1 VS Code extension, Java/Gradle and Spotless editor extensions, and a lightweight Linux desktop. noVNC displays that desktop in a browser tab, so you can use the WPILib simulation GUI without installing WPILib on your laptop. Rendering uses software graphics; a GPU is not required.

Build and test from the terminal:

```sh
./gradlew build
./gradlew test
```

## Run robot simulation in your browser

1. In the Codespace's VS Code terminal, run:

   ```sh
   ./gradlew simulateJava
   ```

   Leave this terminal running. The first simulation run downloads additional native libraries.
2. Open the **Ports** panel next to **Terminal**. Find port **6080**, labeled **Robot simulation desktop (noVNC)**, and click its globe icon (**Open in Browser**). If it is missing, choose **Add Port** and enter `6080`. Use this forwarded address instead of typing `localhost:6080` on your laptop.
3. In the new browser tab, click **Connect** and enter the desktop password: `vscode`. Keep the forwarded port's visibility **Private**; the shared desktop password does not replace GitHub's access control.
4. The WPILib simulation window appears inside the Linux desktop. Use its **Robot State** controls to select **Autonomous**, **Teleoperated**, or **Disabled**. Start in **Disabled**, configure your inputs, then select the mode you want to run.
5. To drive with your keyboard, drag **Keyboard 0** from **System Joysticks** into joystick slot **0**. Open **DS → Keyboard 0 Settings** and configure axes/buttons for the Xbox-controller bindings in `RobotContainer.java`: left Y (axis 1) drives forward/back, left X (axis 0) strafes, and right X (axis 4) turns. For example, map W/S to negative/positive axis 1, A/D to negative/positive axis 0, and Q/E to negative/positive axis 4. Click inside the simulation window so it receives your keystrokes. A USB controller connected to your laptop is not automatically passed through to the Codespace.
6. Inspect the **NetworkTables** view for published robot and mechanism telemetry. Subsystem physics runs in simulation, including the swerve drivetrain, flywheels, hood, turret, and intake arm. Camera observations and real hardware are not automatically simulated.
7. To stop, select **Disabled**, then press **Ctrl+C** in the terminal running Gradle. Closing the noVNC tab only disconnects the viewer; it does not stop the robot program. Stop your Codespace from GitHub when you finish to stop compute usage.

See the [WPILib simulation GUI guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html) for keyboard mappings and other controls, and the [desktop-lite documentation](https://github.com/devcontainers/features/blob/main/src/desktop-lite/README.md) for the browser desktop.

## Troubleshooting

- **Setup or dependency download failed:** rerun `./gradlew --no-daemon compileJava`, then retry the build or simulation command.
- **Port 6080 will not open:** confirm the container was rebuilt after pulling these changes, the Codespace is running, and port 6080 is forwarded. Open it from the **Ports** panel and sign in to GitHub if prompted.
- **Desktop opens but no simulation window appears:** check the Gradle terminal for errors and confirm `./gradlew simulateJava` is still running. A plain Java **Run** button does not configure WPILib simulation. If you changed container settings, rebuild the container.
- **Robot does not respond to the keyboard:** select **Teleoperated**, confirm Keyboard 0 is assigned to joystick slot 0 with the expected axes, and click inside the simulation window to give it keyboard focus.

Use a local WPILib installation and the team robot for robot deployment, hardware access, and testing real sensors or controllers.
