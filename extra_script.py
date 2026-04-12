Import("env")

CXX_ONLY_FLAGS = ["-Wno-volatile", "-fconcepts"]

for flag in CXX_ONLY_FLAGS:
    for flagset in ["CCFLAGS", "CFLAGS"]:
        while flag in env[flagset]:
            env[flagset].remove(flag)
    if flag not in env["CXXFLAGS"]:
        env["CXXFLAGS"].append(flag)
