import pip
import sys

# Check Python version
if not((sys.version_info > (3, ))):
    print("FATAL\tThis Script needs Python 3")
    sys.exit()

# Install Matplotlib if needed
try:
    import matplotlib
except:
    print("matplotlib not installed!\nAuto-installing...")
    pip.main(['install', 'matplotlib'])
    print("Done!")
    # sys.exit()

# Install numpy if needed
try:
    import numpy
except:
    print("numpy not installed!\nAuto-installing...")
    pip.main(['install', 'numpy'])
    print("Done!")
    # sys.exit()
