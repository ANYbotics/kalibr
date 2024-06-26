#include <aslam_imgproc/PinholeUndistorter.hpp>
#include <aslam_cv_python/exportPinholeUndistorter.hpp>
#include <sm_numpy_eigen/boost_python_headers.hpp>

using namespace boost::python;
using namespace aslam;

void exportPinholeUndistorter() {
  aslam::cameras::exportPinholeUndistorter<cameras::RadialTangentialDistortion, aslam::cameras::NoMask>("PinholeUndistorterNoMask");
  aslam::cameras::exportPinholeUndistorter<cameras::EquidistantDistortion, aslam::cameras::NoMask>("EquidistantPinholeUndistorterNoMask");
  aslam::cameras::exportPinholeUndistorter<cameras::FovDistortion, aslam::cameras::NoMask>("FovPinholeUndistorterNoMask");
}
