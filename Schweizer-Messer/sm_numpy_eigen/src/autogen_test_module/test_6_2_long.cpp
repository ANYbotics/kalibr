#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 6, 2> test_long_6_2(const Eigen::Matrix<boost::int64_t, 6, 2> & M)
{
	return M;
}
void export_long_6_2()
{
	boost::python::def("test_long_6_2",test_long_6_2);
}

