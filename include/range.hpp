#pragma once
#include <algorithm>

namespace EIVA
{
template <typename Scalar>
struct Range
{
    Range(const Scalar &a, const Scalar &b)
        : lower(std::min(a, b))
        , upper(std::max(a, b))
    {
    }
    Range()
        : lower(std::numeric_limits<Scalar>::min())
        , upper(std::numeric_limits<Scalar>::max())
    {
    }
    Scalar lower;
    Scalar upper;
    static Range empty()
    {
        Range<Scalar> result;
        result.lower = std::numeric_limits<Scalar>::infinity();
        result.upper = -std::numeric_limits<Scalar>::infinity();
        return result;
    }
};

template <class T>
bool operator==(const Range<T> &lhs, const Range<T> &rhs)
{
    return std::abs(lhs.lower - rhs.lower) < std::numeric_limits<T>::epsilon() && std::abs(lhs.upper - rhs.upper) < std::numeric_limits<T>::epsilon();
}

template <class T>
bool operator!=(const Range<T> &lhs, const Range<T> &rhs)
{
    return !(rhs == lhs);
}

template <class T, class TR>
constexpr const T clamp(const T &v, const Range<TR> &r)
{
    return (v < r.lower) ? r.lower : (r.upper < v) ? r.upper : v;
};

template <class T>
constexpr Range<T> join(const T &v, const Range<T> &r)
{
    return Range<T>(std::min(v, r.lower), std::max(r.upper, v));
};

template <class T>
constexpr bool isIn(const T &v, const Range<T> &r)
{
    return clamp(v, r) == v;
};

template <class T>
constexpr Range<T> clamp(const Range<T> &rhs, const Range<T> &lhs)
{
    return Range<T>(clamp(lhs.lower, rhs), clamp(lhs.upper, rhs));
};

template <class T>
constexpr const T median(const Range<T> &rhs)
{
    return (rhs.upper + rhs.lower) * 0.5;
};

} // namespace EIVA