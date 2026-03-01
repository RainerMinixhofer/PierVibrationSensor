# extra_script.py
from SCons.Script import Import, Builder, AlwaysBuild
import subprocess
import os
import glob

Import("env")


def add_flags_for_main_cpp(source, target, env):
    for src in source:
        if any(src.name.endswith(name) for name in ["main.cpp", "tests.cpp"]):
            env.AppendUnique(CCFLAGS=["-Wall -Wconversion -Wshadow -Wextra -Wunused"])


def generate_version_header(target, source, env):
    """Generate version.h with current git commit hash"""
    version_file = os.path.join(env["PROJECT_DIR"], "include", "version.h")

    try:
        # Get the short commit hash
        commit_hash = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()

        # Get the full commit hash for reference
        full_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()

        # Get commit date
        commit_date = subprocess.check_output(
            ["git", "log", "-1", "--format=%ai"],
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()

        # Check if there are uncommitted changes
        try:
            subprocess.check_output(
                ["git", "diff", "--quiet"],
                cwd=env["PROJECT_DIR"],
                stderr=subprocess.DEVNULL,
            )
            has_changes = False
        except subprocess.CalledProcessError:
            has_changes = True

        dirty_flag = " (dirty)" if has_changes else ""

    except (subprocess.CalledProcessError, FileNotFoundError):
        # Git not available or not a git repository
        commit_hash = "unknown"
        full_hash = "unknown"
        commit_date = "unknown"
        dirty_flag = ""

    # Generate version header
    version_content = f"""#ifndef VERSION_H
#define VERSION_H

#define GIT_COMMIT_SHORT "{commit_hash}{dirty_flag}"
#define GIT_COMMIT_FULL "{full_hash}"
#define GIT_COMMIT_DATE "{commit_date}"

#endif // VERSION_H
"""

    # Write the header file
    try:
        os.makedirs(os.path.dirname(version_file), exist_ok=True)
        with open(version_file, "w") as f:
            f.write(version_content)
        print(f"Generated {version_file} with commit: {commit_hash}{dirty_flag}")
    except Exception as e:
        print(f"Warning: Could not generate version header: {e}")


# Generate version header before compilation
env.AddPreAction("$BUILD_DIR/src/main.cpp.o", generate_version_header)

env.AddBuildMiddleware(add_flags_for_main_cpp)  # type: ignore


def compile_pio_files(source, target, env):
    """Compile .pio files to .pio.h headers using pioasm"""
    # Find pioasm executable
    platform = env.PioPlatform()
    pioasm = None

    # Try to find pioasm in the platform packages
    packages_dir = platform.get_package_dir("framework-arduino-mbed")
    if packages_dir:
        # Look for pioasm in the package
        pioasm_paths = [
            os.path.join(packages_dir, "tools", "pioasm"),
            os.path.join(packages_dir, "tools", "pioasm.exe"),
        ]
        for path in pioasm_paths:
            if os.path.isfile(path):
                pioasm = path
                break

    # Alternative: try to find in Arduino-Pico package
    if not pioasm:
        packages_dir = platform.get_package_dir("framework-arduinopico")
        if packages_dir:
            pioasm_paths = [
                os.path.join(packages_dir, "tools", "pioasm", "pioasm"),
                os.path.join(packages_dir, "tools", "pioasm", "pioasm.exe"),
                os.path.join(packages_dir, "pico-sdk", "tools", "pioasm", "pioasm"),
                os.path.join(packages_dir, "pico-sdk", "tools", "pioasm", "pioasm.exe"),
            ]
            for path in pioasm_paths:
                if os.path.isfile(path):
                    pioasm = path
                    break

    # Fall back to system pioasm
    if not pioasm:
        pioasm = "pioasm"

    # Find all .pio files in test directories
    test_dirs = glob.glob(os.path.join(env["PROJECT_DIR"], "test", "*"))
    for test_dir in test_dirs:
        if os.path.isdir(test_dir):
            pio_files = glob.glob(os.path.join(test_dir, "*.pio"))
            for pio_file in pio_files:
                output_file = pio_file + ".h"

                # Check if we need to recompile
                if os.path.exists(output_file):
                    pio_mtime = os.path.getmtime(pio_file)
                    output_mtime = os.path.getmtime(output_file)
                    if output_mtime >= pio_mtime:
                        continue  # Output is up to date

                # Compile PIO file
                print(f"Compiling PIO: {pio_file} -> {output_file}")
                try:
                    result = subprocess.run(
                        [pioasm, "-o", "c-sdk", pio_file, output_file],
                        capture_output=True,
                        text=True,
                        check=True,
                    )
                    if result.stdout:
                        print(result.stdout)
                except subprocess.CalledProcessError as e:
                    print(f"Error compiling {pio_file}:")
                    print(e.stderr)
                    raise
                except FileNotFoundError:
                    print(f"Warning: pioasm not found, skipping PIO compilation")
                    print(f"Please ensure pioasm is in your PATH or install pico-sdk")
                    break


# Add PIO compilation as a pre-build action
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", compile_pio_files)
