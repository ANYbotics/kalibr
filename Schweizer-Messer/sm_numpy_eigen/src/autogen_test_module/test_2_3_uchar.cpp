#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 2, 3> test_uchar_2_3(const Eigen::Matrix<boost::uint8_t, 2, 3> & M)
{
	return M;
}
void export_uchar_2_3()
{
	boost::python::def("test_uchar_2_3",test_uchar_2_3);
}

