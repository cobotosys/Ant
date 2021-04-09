
//
// Created by cobot on 2020/12/4.
//

#include "StatePlaner.h"
#include "boost/make_shared.hpp"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include "logger/Logger.h"
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/prm/SPARS.h>
StatePlaner::StatePlaner(int dims) : _dims(dims) {
    _stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(_dims);
}

void StatePlaner::setStateChecker(boost::function<bool(rw::math::Q&)> stateChecker) {
    _stateChecker = stateChecker;
}

void StatePlaner::setBounds(rw::math::Q bounds) {
//    _stateSpace->setBounds(bounds);
    ompl::base::RealVectorBounds rvb(_dims);
    for (int i = 0; i < bounds.size(); i++) {
        if (i % 2 == 0) {
            rvb.setLow(i / 2, bounds(i));
        } else {
            rvb.setHigh(i / 2, bounds(i));
        }
    }
    _stateSpace->setBounds(rvb);
}

rw::trajectory::QPath StatePlaner::solve(double time, rw::math::Q start, rw::math::Q goal, double dis) {
    std::shared_ptr<ompl::base::SpaceInformation> si(std::make_shared<ompl::base::SpaceInformation>(_stateSpace));
    si->setStateValidityChecker([&](const ompl::base::State *omplState) -> bool {
        // cast the abstract state type to the type we expect
        const auto *rvstate = omplState->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> values;
        for (int i = 0; i < _dims; i++) {
            values.push_back(rvstate->values[i]);
        }
        rw::math::Q q(values);
        return _stateChecker(q);
    });
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> startState(_stateSpace);
    for (int i = 0; i < start.size(); i++) {
        startState->values[i] = start(i);
    }
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goalState(_stateSpace);
    for (int i = 0; i < goal.size(); i++) {
        goalState->values[i] = goal(i);
    }
    std::shared_ptr<ompl::base::ProblemDefinition> pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(startState, goalState);
    pdef->setOptimizationObjective(
            ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si)));
    std::shared_ptr<ompl::geometric::RRTConnect> planner(std::make_shared<ompl::geometric::RRTConnect>(si));
//    std::shared_ptr<ompl::geometric::TRRT> planner(std::make_shared<ompl::geometric::TRRT>(si));
//    std::shared_ptr<ompl::geometric::BFMT> planner(std::make_shared<ompl::geometric::BFMT>(si));
//    std::shared_ptr<ompl::geometric::RRTstar> planner(std::make_shared<ompl::geometric::RRTstar>(si));
//    std::shared_ptr<ompl::geometric::BITstar> planner(std::make_shared<ompl::geometric::BITstar>(si));
//    std::shared_ptr<ompl::geometric::ABITstar> planner(std::make_shared<ompl::geometric::ABITstar>(si));
//    std::shared_ptr<ompl::geometric::AITstar> planner(std::make_shared<ompl::geometric::AITstar>(si));
//    std::shared_ptr<ompl::geometric::LBTRRT> planner(std::make_shared<ompl::geometric::LBTRRT>(si));

//    std::shared_ptr<ompl::geometric::SPARS> planner(std::make_shared<ompl::geometric::SPARS>(si));

//    std::shared_ptr<ompl::geometric::PRMstar> planner(std::make_shared<ompl::geometric::PRMstar>(si));
//    planner->setDefaultConnectionStrategy();

    planner->setProblemDefinition(pdef);
    planner->setRange(dis);
    planner->setup();
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(time);
    rw::trajectory::QPath qpath;
    if (solved) {
        LOG_INFO << "getSolutionCount " << pdef->getSolutionCount();
        ompl::base::PathPtr path = pdef->getSolutionPath();
        LOG_INFO << "getSolutionCount " << pdef->getSolutionCount();
        auto rawPath = path->as<ompl::geometric::PathGeometric>();
        LOG_INFO << "getStateCount  " << rawPath->getStateCount();
        for (int i = 0; i < rawPath->getStateCount(); i++) {
//            cobotStudioApi.setQ(getQ().toStdVector(), "ABB_IRB_1200_5_90.1");
            const auto *rvstate = rawPath->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> values;
            for (int i = 0; i < _dims; i++) {
                values.push_back(rvstate->values[i]);
            }
            qpath.push_back({values});
        }
    }
    return qpath;
}