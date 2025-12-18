# extra_script.py
from SCons.Script import Import, Builder

Import("env")

def add_flags_for_main_cpp(source, target, env):
    for src in source:
        if any(src.name.endswith(name) for name in ["main.cpp", "tests.cpp"]):
            env.AppendUnique(CCFLAGS=["-Wall -Wconversion -Wshadow -Wextra -Wunused"])

env.AddBuildMiddleware(add_flags_for_main_cpp) # type: ignore