#include "starq/planning/implicit_grid.hpp"

namespace starq::planning
{

    ImplicitGrid::ImplicitGrid(const VectorX &dx) : dx_(dx) {}

    GridKey ImplicitGrid::getKey(const VectorX &x)
    {
        std::vector<int> key;
        key.reserve(x.size());
        for (int i = 0; i < x.size(); i++)
        {
            if (dx_(i) > 0.0)
            {
                key.push_back(static_cast<int>(std::round(x(i) / dx_(i))));
            }
        }
        return key;
    }

    VectorX ImplicitGrid::getState(const GridKey &k, const VectorX &xr)
    {
        VectorX x(xr.size());
        for (int i = 0, j = 0; i < xr.size() && j < int(k.size()); i++)
        {
            if (dx_(i) > 0.0)
            {
                x(i) = static_cast<Float>(k[j]) * dx_(i);
                j++;
            }
            else
            {
                x(i) = xr(i);
            }
        }
        return x;
    }

    Node *ImplicitGrid::getNode(const GridKey &k)
    {
        const auto it = grid_.find(k);
        if (it != grid_.end())
        {
            return it->second.get();
        }
        return nullptr;
    }

    Node *ImplicitGrid::getNode(const VectorX &x)
    {
        auto k = getKey(x);
        auto node = getNode(k);
        if (!node)
        {
            node = createNode(k, x);
        }
        return node;
    }

    Node *ImplicitGrid::createNode(const GridKey &k, const VectorX &xr)
    {
        const VectorX x = getState(k, xr);
        grid_[k] = std::make_unique<Node>();
        grid_[k]->x = x;
        return grid_[k].get();
    }

    std::vector<Node*> ImplicitGrid::getNodes() const
    {
        std::vector<Node*> nodes;
        nodes.reserve(grid_.size());
        for (const auto &it : grid_)
        {
            nodes.push_back(it.second.get());
        }
        return nodes;
    }

}