#include <sm_numpy_eigen/boost_python_headers.hpp>
#include <aslam_cameras/cameras/RollingShutter.hpp>
#include <aslam_cameras/cameras/GlobalShutter.hpp>
#include <boost/serialization/nvp.hpp>
#include <aslam_time/Time.hpp>
#include <sm_python/boost_serialization_pickle.hpp>
#include <sm_python/unique_register_ptr_to_python.hpp>

//#include <aslam_backend_python/ExportDesignVariableAdapter.hpp>

using namespace boost::python;
using namespace aslam::cameras;
using namespace aslam;

template<typename T>
inline Eigen::MatrixXd getParameters(T * D) {

  Eigen::MatrixXd P;
  D->getParameters(P);
  return P;

}

template<typename S>
Duration temporalOffset(const S * shutter, const Eigen::VectorXd & keypoint) {
  return shutter->temporalOffset(keypoint);
}

template<typename C, typename T>
void exportGenericShutterFunctions(T & shutter) {
  shutter.def("temporalOffset", &temporalOffset<C>,
              "Returns the Temporal Offset of a given Keypoint");
  shutter.def("setParameters", &C::setParameters);
  shutter.def("getParameters", &getParameters<C>);
  shutter.def_pickle(sm::python::pickle_suite<C>());
}

void exportGlobalShutter(std::string name) {

  class_<GlobalShutter, boost::shared_ptr<GlobalShutter> > globalShutter(
      name.c_str(), init<>());
  globalShutter.def(init<>((name + "()").c_str()));

  sm::python::unique_register_ptr_to_python<boost::shared_ptr<GlobalShutter> >();

  exportGenericShutterFunctions<GlobalShutter>(globalShutter);
}

void exportRollingShutter(std::string name) {

  class_<RollingShutter, boost::shared_ptr<RollingShutter> > rollingShutter(
      name.c_str(), init<>());
  rollingShutter.def(init<>((name + "()").c_str())).def(
      init<double>((name + "(double lineDelay)").c_str())).def(
      "lineDelay", &RollingShutter::lineDelay,
      "Returns the Line Delay of the Rolling Shutter");

  sm::python::unique_register_ptr_to_python<boost::shared_ptr<RollingShutter> >();

  exportGenericShutterFunctions<RollingShutter>(rollingShutter);
}

void exportCameraShutters() {

  using namespace boost::python;
  using namespace aslam::cameras;

  exportGlobalShutter("GlobalShutter");
  exportRollingShutter("RollingShutter");

  //aslam::python::exportDesignVariableAdapter< GlobalShutter >("GlobalShutterDesignVariable");
  //aslam::python::exportDesignVariableAdapter< RollingShutter >("RollingShutterDesignVariable");

}
