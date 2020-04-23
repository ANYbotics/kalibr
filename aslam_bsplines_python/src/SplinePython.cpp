#include <sm_numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>


void import_bspline_python();
void import_bspline_pose_python();

BOOST_PYTHON_MODULE(libaslam_bsplines_python)
{
	import_bspline_python();
	import_bspline_pose_python();
}
