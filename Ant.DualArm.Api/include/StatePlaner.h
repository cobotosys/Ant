//
// Created by cobot on 2020/12/4.
//
#pragma once

#include "boost/function.hpp"
#include "memory"
#include "rw/math.hpp"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "rw/trajectory.hpp"

class StatePlaner {
public:
    StatePlaner(int dims);

    void setStateChecker(boost::function<bool(rw::math::Q&)> stateChecker);

    void setBounds(rw::math::Q bounds);

    rw::trajectory::QPath solve(double time, rw::math::Q start, rw::math::Q goal, double dis);

private:
    /**
     * 状态检测器
     */
    boost::function<bool(rw::math::Q&)> _stateChecker;
    std::shared_ptr<ompl::base::RealVectorStateSpace> _stateSpace;
    int _dims;
};