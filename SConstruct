#!/usr/bin/env python
import os
import sys

libname = "Drive"
projectdir = "project"

localEnv = Environment(tools=["default"], PLATFORM="")

env = localEnv.Clone()

if not (os.path.isdir("godot-cpp") and os.listdir("godot-cpp")):
    print("no godot-cpp")
    sys.exit(1)

env = SConscript("godot-cpp/SConstruct", {"env": env})

env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp")

suffix = env['suffix'].replace(".dev", "").replace(".universal", "")

lib_filename = "{}{}{}{}".format(env.subst('$SHLIBPREFIX'), libname, suffix, env.subst('$SHLIBSUFFIX'))

library = env.SharedLibrary(
    "bin/{}/{}".format(env['platform'], lib_filename),
    source=sources,
)

copy = env.Install("{}/bin/{}/".format(projectdir, env["platform"]), library)

default_args = [library, copy]
Default(*default_args)
