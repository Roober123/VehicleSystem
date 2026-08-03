#!/usr/bin/env python
# pyright: reportUndefinedVariable=false
import os
import sys

from methods import print_error
from SCons.Variables import BoolVariable


libname = "VehicleSystem"
projectdir = "project"

localEnv = Environment(tools=["default"], PLATFORM="")

# Build profiles can be used to decrease compile times.
# You can either specify "disabled_classes", OR
# explicitly specify "enabled_classes" which disables all other classes.
# Modify the example file as needed and uncomment the line below or
# manually specify the build_profile parameter when running SCons.

# localEnv["build_profile"] = "build_profile.json"

customs = ["custom.py"]
customs = [os.path.abspath(path) for path in customs]

opts = Variables(customs, ARGUMENTS)
opts.Add(
    BoolVariable(
        "tests",
        "Build the debug extension with the deterministic drivetrain regression runner",
        False,
    )
)
opts.Update(localEnv)

Help(opts.GenerateHelpText(localEnv))

run_regression_tests = bool(localEnv.get("tests", False))
env = localEnv.Clone()
if "tests" in env:
    del env["tests"]

if not (os.path.isdir("godot-cpp") and os.listdir("godot-cpp")):
    print_error("""godot-cpp is not available within this folder, as Git submodules haven't been initialized.
Run the following command to download godot-cpp:

    git submodule update --init --recursive""")
    sys.exit(1)

tests_argument = ARGUMENTS.pop("tests", None)
try:
    godot_cpp_env = SConscript("godot-cpp/SConstruct", {"env": env, "customs": customs})
finally:
    if tests_argument is not None:
        ARGUMENTS["tests"] = tests_argument

# Keep the environment returned by godot-cpp immutable.  Its builders and
# archive nodes are shared by both VehicleSystem variants; extension-specific
# defines must never leak back into that construction graph.
production_env = godot_cpp_env.Clone()
production_env.Append(CPPPATH=["src/"])

all_sources = Glob("src/*.cpp") + Glob("src/**/*.cpp")

if production_env["target"] != "template_debug" and run_regression_tests:
    print_error("tests=1 is supported only with target=template_debug")
    sys.exit(2)

if production_env["target"] in ["editor", "template_debug"]:
    try:
        doc_data = production_env.GodotCPPDocData(
            "src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml")
        )
        all_sources.append(doc_data)
    except AttributeError:
        print("Not including class reference as we're targeting a pre-4.3 baseline.")

if production_env["target"] != "template_debug":
    # Keep release/editor targets on their original canonical graph.  The
    # variant split is intentionally limited to template_debug.
    suffix = production_env["suffix"].replace(".dev", "").replace(".universal", "")
    lib_filename = "{}{}{}{}".format(
        production_env.subst("$SHLIBPREFIX"),
        libname,
        suffix,
        production_env.subst("$SHLIBSUFFIX"),
    )
    library = production_env.SharedLibrary(
        "bin/{}/{}".format(production_env["platform"], lib_filename),
        source=all_sources,
    )
    copy = production_env.Install(
        "{}/bin/{}/".format(projectdir, production_env["platform"]), library
    )
    Default(library, copy)
else:
    register_source = File("src/register_types.cpp")
    production_sources = [source for source in all_sources if source != register_source]

    # Build the source-stable production objects once.  Both debug DLL variants
    # consume these exact nodes, so switching `tests` does not invalidate them.
    production_objects = production_env.SharedObject(source=production_sources)

    normal_register_object = production_env.SharedObject(source=register_source)
    normal_sources = production_objects + [normal_register_object]

    test_env = production_env.Clone()
    if run_regression_tests:
        test_env.Append(CPPDEFINES=["VEHICLE_SYSTEM_REGRESSION_TESTS"])
    # Keep test-only objects separate even on toolchains whose default
    # shared-object suffix is just `.os`.
    test_env["SHOBJSUFFIX"] = ".tests" + test_env.subst("$SHOBJSUFFIX")
    test_objects = test_env.SharedObject(
        source=[register_source] + Glob("Test/*.cpp")
    )
    test_sources = production_objects + test_objects

    # .dev doesn't inhibit compatibility, so we don't need to key it.
    # .universal just means "compatible with all relevant arches" so we don't need to key it.
    suffix = production_env["suffix"].replace(".dev", "").replace(".universal", "")
    shlib_prefix = production_env.subst("$SHLIBPREFIX")
    shlib_suffix = production_env.subst("$SHLIBSUFFIX")
    base_filename = "{}{}{}".format(shlib_prefix, libname, suffix)

    normal_filename = base_filename + ".normal" + shlib_suffix
    test_filename = base_filename + ".tests" + shlib_suffix
    canonical_filename = base_filename + shlib_suffix

    normal_library = production_env.SharedLibrary(
        "bin/{}/{}".format(production_env["platform"], normal_filename),
        source=normal_sources,
    )
    test_library = test_env.SharedLibrary(
        "bin/{}/{}".format(test_env["platform"], test_filename),
        source=test_sources,
    )

    selected_library = test_library if run_regression_tests else normal_library
    canonical_path = "{}/bin/{}/{}".format(
        projectdir, production_env["platform"], canonical_filename
    )
    copy = production_env.InstallAs(canonical_path, selected_library)

    Default(selected_library, copy)
