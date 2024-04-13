#ifndef ASLAM_CAMERAS_HPP
#define ASLAM_CAMERAS_HPP

// The camera geometery mother class
#include <aslam_cameras/cameras/CameraGeometry.hpp>

// Projection models
#include <aslam_cameras/cameras/PinholeProjection.hpp>
#include <aslam_cameras/cameras/OmniProjection.hpp>
#include <aslam_cameras/cameras/ExtendedUnifiedProjection.hpp>
#include <aslam_cameras/cameras/DoubleSphereProjection.hpp>
#include <aslam_cameras/cameras/DepthProjection.hpp>

// Distortion models
#include <aslam_cameras/cameras/NoDistortion.hpp>
#include <aslam_cameras/cameras/RadialTangentialDistortion.hpp>
#include <aslam_cameras/cameras/EquidistantDistortion.hpp>
#include <aslam_cameras/cameras/FovDistortion.hpp>

// Masks
#include <aslam_cameras/cameras/NoMask.hpp>
#include <aslam_cameras/cameras/ImageMask.hpp>

// Shutter models
#include <aslam_cameras/cameras/RollingShutter.hpp>
#include <aslam_cameras/cameras/GlobalShutter.hpp>

namespace aslam {
namespace cameras {

typedef CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter, NoMask> PinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<FovDistortion>, GlobalShutter,
    NoMask> FovDistortedPinholeCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, NoMask> OmniCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<FovDistortion>, GlobalShutter,
    NoMask> FovDistortedOmniCameraGeometry;

typedef CameraGeometry<ExtendedUnifiedProjection<NoDistortion>, GlobalShutter, NoMask> ExtendedUnifiedCameraGeometry;
typedef CameraGeometry<DoubleSphereProjection<NoDistortion>, GlobalShutter, NoMask> DoubleSphereCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter, NoMask> PinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    RollingShutter, NoMask> DistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, RollingShutter,
    NoMask> EquidistantDistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<FovDistortion>, RollingShutter,
    NoMask> FovDistortedPinholeRsCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, NoMask> OmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    RollingShutter, NoMask> DistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, RollingShutter,
    NoMask> EquidistantDistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<FovDistortion>, RollingShutter,
    NoMask> FovDistortedOmniRsCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter, ImageMask> MaskedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    GlobalShutter, ImageMask> MaskedDistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, GlobalShutter,
    ImageMask> MaskedEquidistantDistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<FovDistortion>, GlobalShutter,
    ImageMask> MaskedFovDistortedPinholeCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, ImageMask> MaskedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    GlobalShutter, ImageMask> MaskedDistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
    ImageMask> MaskedEquidistantDistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<FovDistortion>, GlobalShutter,
    ImageMask> MaskedFovDistortedOmniCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
    ImageMask> MaskedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    RollingShutter, ImageMask> MaskedDistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, RollingShutter,
    ImageMask> MaskedEquidistantDistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<FovDistortion>, RollingShutter,
    ImageMask> MaskedFovDistortedPinholeRsCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, ImageMask> MaskedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    RollingShutter, ImageMask> MaskedDistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, RollingShutter,
    ImageMask> MaskedEquidistantDistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<FovDistortion>, RollingShutter,
    ImageMask> MaskedFovDistortedOmniRsCameraGeometry;

typedef CameraGeometry<DepthProjection<NoDistortion>, GlobalShutter, NoMask> DepthCameraGeometry;
typedef CameraGeometry<DepthProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedDepthCameraGeometry;
typedef CameraGeometry<DepthProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedDepthCameraGeometry;
typedef CameraGeometry<DepthProjection<FovDistortion>, GlobalShutter,
    NoMask> FovDistortedDepthCameraGeometry;

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_CAMERAS_HPP */
