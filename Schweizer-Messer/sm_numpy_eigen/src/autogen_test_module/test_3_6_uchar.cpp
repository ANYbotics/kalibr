#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 3, 6> test_uchar_3_6(const Eigen::Matrix<boost::uint8_t, 3, 6> & M)
{
	return M;
}
void export_uchar_3_6()
{
	boost::python::def("test_uchar_3_6",test_uchar_3_6);
}

