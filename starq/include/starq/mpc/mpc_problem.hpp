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

        VectorXf &getXref(const size_t &k);

        MatrixXf &getQ(const size_t &k);

        MatrixXf &getR(const size_t &k);

        MatrixXf &getA(const size_t &k);

        MatrixXf &getB(const size_t &k);

        MatrixXf &getC(const size_t &k);

        VectorXf &getLower(const size_t &k);

        VectorXf &getUpper(const size_t &k);

    private:
        const MPCConfiguration::Ptr config_;

        std::vector<VectorXf> xref_;
        std::vector<MatrixXf> Q_;
        std::vector<MatrixXf> R_;
        std::vector<MatrixXf> A_;
        std::vector<MatrixXf> B_;
        std::vector<MatrixXf> C_;
        std::vector<VectorXf> lower_;
        std::vector<VectorXf> upper_;

        void computeXref();
        void computeQ();
        void computeR();
        void computeA();
        void computeB();
        void computeC();
        void computeLower();
        void computeUpper();

        Matrix3f getSkewSymmetricMatrix(const Vector3f &v) const;
    };

}

#endif