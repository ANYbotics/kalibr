#include <sm_numpy_eigen/boost_python_headers.hpp>
#include <aslam_cameras/cameras.hpp>
#include <aslam_cameras/FrameBuilder.hpp>
#include <aslam_cameras/Frame.hpp>

void exportFrameBuilder() {
  using namespace boost::python;
  using namespace aslam;
  using namespace aslam::cameras;

  def("createFrameBuilder", &FrameBuilder::createFrameBuilder);

  class_<FrameBuilder, boost::shared_ptr<FrameBuilder>, boost::noncopyable>(
      "FrameBuilder", no_init).def("updateConfiguration",
                                   &FrameBuilder::updateConfiguration).def(
      "buildFrame", &FrameBuilder::buildFrame);
}
