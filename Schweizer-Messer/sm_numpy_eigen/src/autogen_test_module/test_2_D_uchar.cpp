#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 2, Eigen::Dynamic> test_uchar_2_D(const Eigen::Matrix<boost::uint8_t, 2, Eigen::Dynamic> & M)
{
	return M;
}
void export_uchar_2_D()
{
	boost::python::def("test_uchar_2_D",test_uchar_2_D);
}

