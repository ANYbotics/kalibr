#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 4, 4> test_uchar_4_4(const Eigen::Matrix<boost::uint8_t, 4, 4> & M)
{
	return M;
}
void export_uchar_4_4()
{
	boost::python::def("test_uchar_4_4",test_uchar_4_4);
}
