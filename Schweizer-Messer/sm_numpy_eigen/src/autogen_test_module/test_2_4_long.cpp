#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 2, 4> test_long_2_4(const Eigen::Matrix<boost::int64_t, 2, 4> & M)
{
	return M;
}
void export_long_2_4()
{
	boost::python::def("test_long_2_4",test_long_2_4);
}

