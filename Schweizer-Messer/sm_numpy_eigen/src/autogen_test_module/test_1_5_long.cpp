#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 1, 5> test_long_1_5(const Eigen::Matrix<boost::int64_t, 1, 5> & M)
{
	return M;
}
void export_long_1_5()
{
	boost::python::def("test_long_1_5",test_long_1_5);
}

