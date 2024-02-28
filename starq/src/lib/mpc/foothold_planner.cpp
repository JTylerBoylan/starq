#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    FootholdPlanner::FootholdPlanner(std::vector<LegController::Ptr> legs,
                                     starq::slam::Localization::Ptr localization)
        : legs_(legs),
          localization_(localization)
    {
    }

    FootholdPlanner::~FootholdPlanner()
    {
    }

    bool FootholdPlanner::configure(MPCConfiguration &config) const
    {
        // TODO
        return true;
    }

}