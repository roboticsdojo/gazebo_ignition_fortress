# WSL2 GPU Graphics Fixes for Gazebo Ignition

## Overview

This document describes the specific GPU-related environment variables added to `src/ppp_bot/launch/launch_ign.launch.py` to resolve graphics rendering issues when running Gazebo Ignition Fortress on WSL2.

## Changes Made

The following environment variables were added to lines 69-76 in `launch_ign.launch.py`:

```python
SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
SetEnvironmentVariable('IGN_RENDERING_ENGINE_PATH', '/usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins'),
SetEnvironmentVariable('IGN_RENDERING_ENGINE', 'ogre2'),
SetEnvironmentVariable('DISPLAY', ':0'),
SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
```

## Environment Variable Explanations

### 1. `LIBGL_ALWAYS_SOFTWARE=1`
- **Purpose**: Forces OpenGL to use software rendering instead of hardware acceleration
- **Why Needed**: WSL2's graphics stack often has compatibility issues with hardware OpenGL
- **Effect**: Uses CPU-based rendering, which is more stable but slower

### 2. `MESA_GL_VERSION_OVERRIDE=3.3`
- **Purpose**: Sets the OpenGL version to 3.3 for compatibility
- **Why Needed**: Ensures consistent OpenGL version across different WSL2 setups
- **Effect**: Prevents version mismatch errors in the rendering pipeline

### 3. `MESA_GLSL_VERSION_OVERRIDE=330`
- **Purpose**: Sets the GLSL (OpenGL Shading Language) version to 330
- **Why Needed**: Matches the OpenGL 3.3 version for shader compatibility
- **Effect**: Ensures shaders compile and run correctly

### 4. `IGN_RENDERING_ENGINE_PATH=/usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins`
- **Purpose**: Specifies the path to Ignition rendering engine plugins
- **Why Needed**: Ensures Gazebo Ignition can find the correct rendering plugins
- **Effect**: Enables proper loading of OGRE2 rendering engine

### 5. `IGN_RENDERING_ENGINE=ogre2`
- **Purpose**: Explicitly sets the rendering engine to OGRE2
- **Why Needed**: Forces use of the OGRE2 renderer which is more stable in WSL2
- **Effect**: Prevents fallback to incompatible rendering engines

### 6. `DISPLAY=:0`
- **Purpose**: Sets the X11 display to the default display
- **Why Needed**: Ensures GUI applications know where to render
- **Effect**: Enables proper window creation and display

### 7. `QT_X11_NO_MITSHM=1`
- **Purpose**: Disables MIT-SHM (MIT Shared Memory) for Qt applications
- **Why Needed**: WSL2 has issues with shared memory for X11 applications
- **Effect**: Prevents Qt-based GUI crashes (Gazebo GUI, RViz)

## Expected Results

After adding these environment variables, you should experience:

1. **Elimination of OGRE UnimplementedException errors**
2. **Successful Gazebo Ignition startup**
3. **Proper GUI rendering for both Gazebo and RViz**
4. **Stable simulation performance**

## Performance Considerations

- **Software Rendering**: Using `LIBGL_ALWAYS_SOFTWARE=1` will result in slower graphics performance
- **CPU Usage**: Higher CPU usage due to software rendering
- **Memory**: Potentially higher memory usage for rendering operations

## Alternative Approaches

If performance becomes an issue, consider:

1. **WSL2 with GPU Support**: Configure WSL2 with NVIDIA GPU support for hardware acceleration
2. **Headless Mode**: Run without GUI for pure simulation testing
3. **Native Linux**: Use a native Ubuntu installation for optimal performance

## Verification

To verify the fixes are working:

```bash
# Check if environment variables are set
echo $LIBGL_ALWAYS_SOFTWARE
echo $MESA_GL_VERSION_OVERRIDE
echo $IGN_RENDERING_ENGINE

# Test basic OpenGL
glxinfo | grep "OpenGL renderer"

# Run the launch file
ros2 launch ppp_bot launch_sim.launch.py
```

## Troubleshooting

If issues persist:

1. **Check WSL2 Graphics Setup**: Ensure WSLg (Windows 11) or VcXsrv (Windows 10) is properly configured
2. **Verify X11**: Test with `xeyes` or `xclock` to ensure X11 is working
3. **Update WSL2**: Ensure you have the latest WSL2 version
4. **Check Display**: Verify `echo $DISPLAY` returns `:0` 