# extra_script.py
from SCons.Script import Import, Builder
import subprocess
import os

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
            universal_newlines=True
        ).strip()
        
        # Get the full commit hash for reference
        full_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True
        ).strip()
        
        # Get commit date
        commit_date = subprocess.check_output(
            ["git", "log", "-1", "--format=%ai"],
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True
        ).strip()
        
        # Check if there are uncommitted changes
        try:
            subprocess.check_output(
                ["git", "diff", "--quiet"],
                cwd=env["PROJECT_DIR"],
                stderr=subprocess.DEVNULL
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

env.AddBuildMiddleware(add_flags_for_main_cpp) # type: ignore