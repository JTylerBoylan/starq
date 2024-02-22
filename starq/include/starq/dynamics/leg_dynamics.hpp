#ifndef STARQ_DYNAMICS__LEG_DYNAMICS_HPP_
#define STARQ_DYNAMICS__LEG_DYNAMICS_HPP_

#include "eigen3/Eigen/Dense"
#include <functional>

namespace starq::dynamics
{
    using namespace Eigen;

    class Link
    {
    public:
        Link(float m, Matrix3f I,
             std::function<Matrix3f(Vector3f)> R,
             std::function<Matrix3f(Vector3f)> Jv,
             std::function<Matrix3f(Vector3f)> Jw)
            : m_(m), I_(I), R_(R), Jv_(Jv), Jw_(Jw) {}

        Matrix3f M(Vector3f q)
        {
            auto R = R_(q);
            auto Jv = Jv_(q);
            auto Jw = Jw_(q);
            return m_ * Jv.transpose() * Jv + Jw.transpose() * R * I_ * R.transpose()  * Jw;
        }

    private:
        float m_;
        Matrix3f I_;
        std::function<Matrix3f(Vector3f)> R_;
        std::function<Matrix3f(Vector3f)> Jv_;
        std::function<Matrix3f(Vector3f)> Jw_;
    };

}

#endif