# Import the numpy to Eigen type conversion.
import roslib; roslib.load_manifest('sm_numpy_eigen'); import sm_numpy_eigen
# Import the sm library
import roslib; roslib.load_manifest('sm_python'); import sm_python
# Import the aslam backend
import roslib; roslib.load_manifest('aslam_backend'); import aslam_backend_python
import roslib; roslib.load_manifest('aslam_cv_python'); import aslam_cv_python
# Import the the C++ exports from your package library.
from aslam_cv_backend_python.libaslam_cv_backend_python import *
# Import other files in the directory
# from mypyfile import *

# Now build some convenience wrappers
class CameraModel(object):
    pass

class Omni(CameraModel):
    geometry = aslam_cv_python.OmniCameraGeometry
    reprojectionError = OmniReprojectionError
    reprojectionErrorSimple = OmniReprojectionErrorSimple
    designVariable = OmniCameraGeometryDesignVariable
    projectionType = aslam_cv_python.OmniProjection
    distortionType = aslam_cv_python.NoDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.OmniFrame

class DistortedOmni(CameraModel):
    geometry = aslam_cv_python.DistortedOmniCameraGeometry
    reprojectionError = DistortedOmniReprojectionError
    reprojectionErrorSimple = DistortedOmniReprojectionErrorSimple
    designVariable = DistortedOmniCameraGeometryDesignVariable
    projectionType = aslam_cv_python.DistortedOmniProjection
    distortionType = aslam_cv_python.RadialTangentialDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.DistortedOmniFrame

class DistortedOmniRs(CameraModel):
    geometry = aslam_cv_python.DistortedOmniRsCameraGeometry
    reprojectionError = DistortedOmniRsReprojectionError
    reprojectionErrorSimple = DistortedOmniRsReprojectionErrorSimple
    reprojectionErrorAdaptiveCovariance = DistortedOmniRsReprojectionErrorAdaptiveCovariance
    designVariable = DistortedOmniRsCameraGeometryDesignVariable
    projectionType = aslam_cv_python.DistortedOmniProjection
    distortionType = aslam_cv_python.RadialTangentialDistortion
    shutterType = aslam_cv_python.RollingShutter
    frameType = aslam_cv_python.DistortedOmniRsFrame

class DistortedPinhole(CameraModel):
    geometry = aslam_cv_python.DistortedPinholeCameraGeometry
    reprojectionError = DistortedPinholeReprojectionError
    reprojectionErrorSimple = DistortedPinholeReprojectionErrorSimple
    designVariable = DistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv_python.DistortedPinholeProjection
    distortionType = aslam_cv_python.RadialTangentialDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.DistortedPinholeFrame

class DistortedPinholeRs(CameraModel):
    geometry = aslam_cv_python.DistortedPinholeRsCameraGeometry
    reprojectionError = DistortedPinholeRsReprojectionError
    reprojectionErrorSimple = DistortedPinholeRsReprojectionErrorSimple
    reprojectionErrorAdaptiveCovariance = DistortedPinholeRsReprojectionErrorAdaptiveCovariance
    designVariable = DistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv_python.DistortedPinholeProjection
    distortionType = aslam_cv_python.RadialTangentialDistortion
    shutterType = aslam_cv_python.RollingShutter
    frameType = aslam_cv_python.DistortedPinholeRsFrame

class EquidistantPinhole(CameraModel):
    geometry = aslam_cv_python.EquidistantDistortedPinholeCameraGeometry
    reprojectionError = EquidistantDistortedPinholeReprojectionError
    reprojectionErrorSimple = EquidistantDistortedPinholeReprojectionErrorSimple
    designVariable = EquidistantDistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv_python.EquidistantPinholeProjection
    distortionType = aslam_cv_python.EquidistantDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.EquidistantDistortedPinholeFrame

class EquidistantPinholeRs(CameraModel):
    geometry = aslam_cv_python.EquidistantDistortedPinholeRsCameraGeometry
    reprojectionError = EquidistantDistortedPinholeRsReprojectionError
    reprojectionErrorSimple = EquidistantDistortedPinholeRsReprojectionErrorSimple
    reprojectionErrorAdaptiveCovariance = EquidistantDistortedPinholeRsReprojectionErrorAdaptiveCovariance
    designVariable = EquidistantDistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv_python.EquidistantPinholeProjection
    distortionType = aslam_cv_python.EquidistantDistortion
    shutterType = aslam_cv_python.RollingShutter
    frameType = aslam_cv_python.EquidistantPinholeRsFrame

class FovPinhole(CameraModel):
    geometry = aslam_cv_python.FovDistortedPinholeCameraGeometry
    reprojectionError = FovDistortedPinholeReprojectionError
    reprojectionErrorSimple = FovDistortedPinholeReprojectionErrorSimple
    designVariable = FovDistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv_python.FovPinholeProjection
    distortionType = aslam_cv_python.FovDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.FovDistortedPinholeFrame

class ExtendedUnified(CameraModel):
    geometry = aslam_cv_python.ExtendedUnifiedCameraGeometry
    reprojectionError = ExtendedUnifiedReprojectionError
    reprojectionErrorSimple = ExtendedUnifiedReprojectionErrorSimple
    designVariable = ExtendedUnifiedCameraGeometryDesignVariable
    projectionType = aslam_cv_python.ExtendedUnifiedProjection
    distortionType = aslam_cv_python.NoDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.ExtendedUnifiedFrame

class DoubleSphere(CameraModel):
    geometry = aslam_cv_python.DoubleSphereCameraGeometry
    reprojectionError = DoubleSphereReprojectionError
    reprojectionErrorSimple = DoubleSphereReprojectionErrorSimple
    designVariable = DoubleSphereCameraGeometryDesignVariable
    projectionType = aslam_cv_python.DoubleSphereProjection
    distortionType = aslam_cv_python.NoDistortion
    shutterType = aslam_cv_python.GlobalShutter
    frameType = aslam_cv_python.DoubleSphereFrame

