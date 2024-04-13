import aslam_cv_backend_python #needed for errorterm base class wrapper
# This works when you install the debian package ros-noetic-kalibr. If you build it locally, you need to use from libkalibr_errorterms_python import * instead.
# Todo: need to make it consistent in both locally build and debian.
from kalibr.libkalibr_errorterms_python import *
