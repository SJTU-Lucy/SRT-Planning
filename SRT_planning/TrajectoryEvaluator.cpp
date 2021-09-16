#include "TrajectoryEvaluator.h"

TrajectoryEvaluator::TrajectoryEvaluator(std::vector<double>& s,
    std::vector<double>& d,
    std::vector<Curve1d>& lon_trajectory1d_bundle,
    std::vector<Curve1d>& lat_trajectory1d_bundle,
    std::vector<PathPoint>& ptr_reference_line)
{
    // set a cmp function for sort()
    auto cmp = [](TrajectoryDemo a, TrajectoryDemo b)
        ->bool { return a.cost() > b.cost(); };
    init_s = s;
    init_d = d;
    lon_bundle = lon_trajectory1d_bundle;
    lat_bundle = lat_trajectory1d_bundle;
    ref_points = ptr_reference_line;
    for (unsigned int i = 0; i < lon_bundle.size(); ++i)
    {
        for (unsigned int j = 0; j < lat_bundle.size(); ++j)
        {
            TrajectoryDemo tmp(init_s, init_d, lon_bundle[i], lat_bundle[j], ref_points);
            rank.push_back(tmp);
        }
    }
    std::sort(rank.begin(), rank.end(), cmp);
}

bool TrajectoryEvaluator::has_more_trajectory_pairs()
{
    if (rank.size() > 0) return true;
    else return false;
}

int TrajectoryEvaluator::num_of_trajectory_pairs()
{
    return lon_bundle.size() * lat_bundle.size();
}

TrajectoryDemo TrajectoryEvaluator::next_top_trajectory_pair()
{
    TrajectoryDemo ret = rank[rank.size() - 1];
    // ret.lon().print();
    // ret.lat().print();
    rank.pop_back();
    return ret;
}