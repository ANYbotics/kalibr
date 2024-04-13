#include <aslam_cameras/FrameBase.hpp>
// Standard serialization headers
#include <boost/serialization/export.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
// These ones are in sm_boost
#include <sm_boost/portable_binary_iarchive.hpp>
#include <sm_boost/portable_binary_oarchive.hpp>

// This is suggested here:
#include <aslam_cv_serialization/LinkCvSerialization.hpp>

namespace aslam {
int linkCvSerialization() {
  // Stop the compiler from optimizing out the function
  static int x = 0;
  return ++x;

}
}  // namespace aslam

