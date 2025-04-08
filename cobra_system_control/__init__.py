from pathlib import Path
import sys

from cobra_system_control.cobra_log import setup_logging

# for development off the NCB, identify an appropriate directory to save
# logs and other data (e.g. temporary testing data)
if sys.platform.startswith("win32") or sys.platform.startswith("cygwin"):
    COBRA_DIR = Path(Path.home(), "AppData", "Local", "Cobra").resolve()
elif sys.platform.startswith("linux") or sys.platform.startswith("darwin"):
    COBRA_DIR = Path(Path.home(), "cobra").resolve()
else:
    raise SystemError("What OS are you running on???")

COBRA_DIR.mkdir(parents=True, exist_ok=True)

# setup the logging
setup_logging()
