#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, 1> test_uchar_D_1(const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, 1> & M)
{
	return M;
}
void export_uchar_D_1()
{
	boost::python::def("test_uchar_D_1",test_uchar_D_1);
}

