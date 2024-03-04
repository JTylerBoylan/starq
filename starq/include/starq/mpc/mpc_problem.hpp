#ifndef STARQ_MPC__MPC_PROBLEM_HPP_
#define STARQ_MPC__MPC_PROBLEM_HPP_

#include "starq/mpc/mpc_configuration.hpp"

namespace starq::mpc
{

    class MPCProblem
    {
    public:
        using Ptr = std::shared_ptr<MPCProblem>;

        MPCProblem(const MPCConfiguration::Ptr config);

        ~MPCProblem();

        void update();

        MPCConfiguration::Ptr getConfig() const;

        size_t getNx() const;

        size_t getNu() const;

        VectorXf getXref(const size_t &k) const;

        MatrixXf getQ(const size_t &k) const;

        MatrixXf getR(const size_t &k) const;

        MatrixXf getA(const size_t &k) const;

        MatrixXf getB(const size_t &k) const;

        MatrixXf getC(const size_t &k) const;

        VectorXf getLower(const size_t &k) const;

        VectorXf getUpper(const size_t &k) const;

    private:
        const MPCConfiguration::Ptr config_;

        size_t nx_;
        size_t nu_;

        Matrix3f getSkewSymmetricMatrix(const Vector3f &v) const;
    };

}

#endif