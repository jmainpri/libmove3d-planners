#ifndef PR2_MAPPING_HPP
#define PR2_MAPPING_HPP

#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

#include <map>

class pr2_mapping
{
public:
    pr2_mapping(Move3D::Robot* robot);
    Move3D::Trajectory load_trajectory(std::string filename);

private:
    Move3D::Robot* robot_;
    std::map<std::string, std::string> name_map_;
    std::map<std::string, int> pr2_map_;
};

#endif // PR2_MAPPING_HPP
