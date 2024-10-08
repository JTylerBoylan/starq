#ifndef STARQ_PLANNING__IMPLICIT_GRID_HPP_
#define STARQ_PLANNING__IMPLICIT_GRID_HPP_

#include <unordered_map>
#include "starq/planning/planning_types.hpp"

namespace starq::planning
{

    class ImplicitGrid
    {
    public:
        using Ptr = std::shared_ptr<ImplicitGrid>;

        ImplicitGrid(const VectorX &dx);

        GridKey getKey(const VectorX &x);

        VectorX getState(const GridKey &k, const VectorX &xr);

        Node *getNode(const GridKey &k);

        Node *getNode(const VectorX &x);

        Node *createNode(const GridKey &k, const VectorX &xr);

        std::vector<Node *> getNodes() const;

    private:
        struct GridKeyHash
        {
            std::size_t operator()(const std::vector<int> &v) const
            {
                std::size_t seed = v.size();
                for (const auto &i : v)
                {
                    seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
        };

        const VectorX dx_;
        std::unordered_map<GridKey, std::unique_ptr<Node>, GridKeyHash> grid_;
    };

}

#endif