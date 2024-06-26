// It is extremely important to use this header
// if you are using the sm_numpy_eigen interface
#include <sm_numpy_eigen/boost_python_headers.hpp>
#include <sm_python/stl_converters.hpp>
#include <aslam_cv_backend/ScalarExpressionNodeKeypointTime.hpp>

namespace aslam {
  namespace python {

    template<typename CAMERA_T>
    void exportScalarExpressionNodeKeypointTime(std::string name) {
      using namespace boost::python;
      using namespace aslam;

      class_<
        ScalarExpressionNodeKeypointTime<CAMERA_T>,
        boost::shared_ptr<ScalarExpressionNodeKeypointTime<CAMERA_T> >
      >(
            (name + "ScalarExpressionNodeKeypointTime").c_str(),
            init<
              const aslam::Time &,
              const Eigen::VectorXd &,
              boost::shared_ptr<aslam::backend::CameraDesignVariable<CAMERA_T> >
            >(
              "ScalarExpressionNodeKeypointTime(frame timestamp, keypoint, camera design variable)"
            )
          )
          .def("toScalar", &ScalarExpressionNodeKeypointTime<CAMERA_T>::toScalar);
    }
  }
}
