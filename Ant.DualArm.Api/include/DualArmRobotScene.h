//
// Created by cobot on 2021/3/18.
//

#ifndef SPARROWC3TEST_DUALARMROBOTSCENE_H
#define SPARROWC3TEST_DUALARMROBOTSCENE_H

#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"
#include "DualArmRobotPlanner.h"
#include "DualArmRobotDriver.h"
#include "rw/invkin/JacobianIKSolver.hpp"

class DualArmRobotScene {
public:
    DualArmRobotScene();

    bool moveToFreely(rw::math::Transform3D<> left, rw::math::Transform3D<> right);

    bool moveToConstraint(rw::math::Transform3D<> left, rw::math::Transform3D<> right,
                          boost::function<bool(rw::math::Q &)> stateConstraint);


    boost::shared_ptr<DualArmRobotPlanner> _planner;
    boost::shared_ptr<DualArmRobotDriver> _driver;
    rw::models::WorkCell::Ptr _workCell;
};


#endif //SPARROWC3TEST_DUALARMROBOTSCENE_H
