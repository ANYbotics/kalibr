#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 5, 6> test_long_5_6(const Eigen::Matrix<boost::int64_t, 5, 6> & M)
{
	return M;
}
void export_long_5_6()
{
	boost::python::def("test_long_5_6",test_long_5_6);
}

