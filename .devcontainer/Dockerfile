# GradleRIO 2026 builds this project with Java 17.
# PhotonVision JNI needs GLIBCXX_3.4.32, newer than Bookworm's C++ runtime.
FROM mcr.microsoft.com/devcontainers/java:3-17-trixie

# WPILib's simulation GUI uses X11 and OpenGL. Mesa renders in software on
# Codespaces, where the desktop-lite feature supplies the virtual display.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      libgl1 \
      libgl1-mesa-dri \
      libglx-mesa0 \
      libxcursor1 \
      libxi6 \
      libxinerama1 \
      libxrandr2 \
    && rm -rf /var/lib/apt/lists/*

# Keep the WPILib VS Code extension in step with the GradleRIO version in build.gradle.
# Download at image-build time so the VSIX exists before VS Code installs extensions.
ARG WPILIB_VERSION=2026.2.1
RUN mkdir -p /opt/wpilib \
    && curl --fail --location --retry 3 \
      "https://github.com/wpilibsuite/vscode-wpilib/releases/download/v${WPILIB_VERSION}/vscode-wpilib-${WPILIB_VERSION}.vsix" \
      --output /opt/wpilib/vscode-wpilib.vsix \
    && chmod 644 /opt/wpilib/vscode-wpilib.vsix
