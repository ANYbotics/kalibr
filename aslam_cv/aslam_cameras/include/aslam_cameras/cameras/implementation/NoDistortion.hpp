namespace aslam {
namespace cameras {

template<typename DERIVED_Y>
void NoDistortion::distort(const DERIVED_Y & /* y */) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED_Y, 2);
}

template<typename DERIVED_Y, typename DERIVED_JY>
void NoDistortion::distort(
    const DERIVED_Y & /* y */,
    const DERIVED_JY & outJyConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED_Y, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED_JY, 2, 2);
  DERIVED_JY & outJy = const_cast<DERIVED_JY &>(outJyConst);
  outJy.derived().resize(2, 2);
  outJy.setIdentity();
}

template<typename DERIVED>
void NoDistortion::undistort(const DERIVED & /* y */) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED, 2);
}

template<typename DERIVED, typename DERIVED_JY>
void NoDistortion::undistort(
    const DERIVED & /* y */,
    const DERIVED_JY & outJyConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED_JY, 2, 2);
  DERIVED_JY & outJy = const_cast<DERIVED_JY &>(outJyConst);
  outJy.derived().resize(2, 2);
  outJy.setIdentity();
}

template<typename DERIVED_Y, typename DERIVED_JD>
void NoDistortion::distortParameterJacobian(
    const DERIVED_Y & /* imageY */,
    const DERIVED_JD & outJdConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      DERIVED_Y, 2);
  // EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(DERIVED_JD, 2, 2);
  DERIVED_JD & outJd = const_cast<DERIVED_JD &>(outJdConst);
  outJd.derived().resize(2, 0);
  //outJd.setIdentity();
}

}  // namespace cameras
}  // namespace aslam
