#include <Eigen/Core>

#include <sm_numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 2, 5> test_uchar_2_5(const Eigen::Matrix<boost::uint8_t, 2, 5> & M)
{
	return M;
}
void export_uchar_2_5()
{
	boost::python::def("test_uchar_2_5",test_uchar_2_5);
}

