Import("env")
import os
from datetime import datetime

# Stamp every build so the info screen can show when firmware and filesystem were built
stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
project = env["PROJECT_DIR"]

with open(os.path.join(project, "include", "build_info.h"), "w") as f:
    f.write('#pragma once\n#define FIRMWARE_BUILD_TIME "%s"\n' % stamp)

with open(os.path.join(project, "data", "buildinfo.txt"), "w") as f:
    f.write(stamp)
