---
description: Build catkin workspace with catkin build or catkin_make
argument-hint: [package-name] [--clean]
allowed-tools: Bash, Read, Glob
---

Build the ROS catkin workspace. Optional arguments in $ARGUMENTS:
- Package name to build specific package
- `--clean` to clean before building

## Step 1: Detect catkin workspace

```bash
# Check for catkin workspace
WS="${CATKIN_WS:-catkin_ws}"
if [ -d "$WS" ]; then
  echo "WORKSPACE: $WS"
  ls -la $WS/
elif [ -f "package.xml" ]; then
  echo "WORKSPACE: $(pwd) (package root)"
else
  echo "NO_CATKIN_WORKSPACE_FOUND"
fi
```

## Step 2: Detect build system

```bash
# Check if using catkin_tools or catkin_make
WS="${CATKIN_WS:-catkin_ws}"
if [ -d "$WS/.catkin_tools" ]; then
  echo "BUILD_SYSTEM: catkin_tools (catkin build)"
else
  echo "BUILD_SYSTEM: catkin_make"
fi
```

## Step 3: Clean if requested

If $ARGUMENTS contains `--clean`:

```bash
cd ${CATKIN_WS:-catkin_ws}

# For catkin_tools
if [ -d ".catkin_tools" ]; then
  catkin clean -y 2>&1
else
  # For catkin_make - remove build and devel
  rm -rf build devel
  echo "Cleaned build/ and devel/ directories"
fi
```

## Step 4: Build workspace

### Using catkin build (catkin_tools):
```bash
cd ${CATKIN_WS:-catkin_ws}

# Build all or specific package
if [ -n "$PACKAGE_NAME" ]; then
  catkin build $PACKAGE_NAME 2>&1
else
  catkin build 2>&1
fi
```

### Using catkin_make:
```bash
cd ${CATKIN_WS:-catkin_ws}

if [ -n "$PACKAGE_NAME" ]; then
  catkin_make --only-pkg-with-deps $PACKAGE_NAME 2>&1
else
  catkin_make 2>&1
fi
```

## Step 5: Source the workspace

After successful build, remind user to source:

```bash
source ${CATKIN_WS:-catkin_ws}/devel/setup.bash
```

## Output Format

### Build Summary

**Workspace:** /path/to/catkin_ws
**Build System:** catkin build / catkin_make
**Packages Built:** [list]
**Status:** SUCCESS / FAILED

### If errors occurred:
Parse the build output for:
1. **Compilation errors** - Show file:line and error message
2. **Missing dependencies** - Suggest `rosdep install --from-paths src --ignore-src -r -y`
3. **CMake errors** - Show relevant CMake output

### After build:
```
Remember to source the workspace:
source catkin_ws/devel/setup.bash
```
