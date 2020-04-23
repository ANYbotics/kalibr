#include <aslam_cv_serialization/cameras/CameraBaseSerialization.hpp>

// Standard serialization headers
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
// These ones are in sm_boost
#include <sm_boost/portable_binary_iarchive.hpp>
#include <sm_boost/portable_binary_oarchive.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::MaskedOmniRsCameraGeometry);

