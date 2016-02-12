// ... until it will be included:
// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1037

template<typename OtherDerived>
inline Scalar cross2 (const MatrixBase<OtherDerived>& other) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,2)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived,2)

    return this->operator[](0) * other.operator[](1) - other.operator[](0) * this->operator[](1);
}