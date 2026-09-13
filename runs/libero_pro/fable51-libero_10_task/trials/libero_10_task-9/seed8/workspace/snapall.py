from lib import *
import sys
r=Robot("snapall")
for cam in sys.argv[1:]: r.snap(cam)
