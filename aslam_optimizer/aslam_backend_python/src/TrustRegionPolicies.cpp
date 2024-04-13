#include <sm_numpy_eigen/boost_python_headers.hpp>
#include <aslam_backend/backend/Optimizer2.hpp>
#include <aslam_backend/backend/TrustRegionPolicy.hpp>
#include <aslam_backend/backend/GaussNewtonTrustRegionPolicy.hpp>
#include <aslam_backend/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam_backend/backend/DogLegTrustRegionPolicy.hpp>


using namespace boost::python;
using namespace aslam::backend;

void exportTrustRegionPolicies() {

  class_<TrustRegionPolicy, boost::shared_ptr<TrustRegionPolicy>, boost::noncopyable>("TrustRegionPolicy", no_init)
      .def("name", &TrustRegionPolicy::name)
      .def("requiresAugmentedDiagonal", &TrustRegionPolicy::requiresAugmentedDiagonal)
      ;

  // GN
  class_<GaussNewtonTrustRegionPolicy, boost::shared_ptr<GaussNewtonTrustRegionPolicy>, bases< TrustRegionPolicy >, boost::noncopyable >("GaussNewtonTrustRegionPolicy", init<>() )
      ;
  
  // LM
  class_<LevenbergMarquardtTrustRegionPolicy, boost::shared_ptr<LevenbergMarquardtTrustRegionPolicy>, bases< TrustRegionPolicy >, boost::noncopyable >("LevenbergMarquardtTrustRegionPolicy", init<>() )
      .def(init<double>("LevenbergMarquardtTrustRegionPolicy( double initalLambda )"))
      ;

  // DL
  class_<DogLegTrustRegionPolicy, boost::shared_ptr<DogLegTrustRegionPolicy>, bases< TrustRegionPolicy >, boost::noncopyable >("DogLegTrustRegionPolicy", init<>() )
      ;
  
}
