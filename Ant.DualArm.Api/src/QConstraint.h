//
// Created by dev on 2020/8/30.
//

#ifndef PUPPY_MYQCONSTRAINT_H
#define PUPPY_MYQCONSTRAINT_H

#include "rw/pathplanning/QConstraint.hpp"
#include "rw/pathplanning/StateConstraint.hpp"
#include "rw/models/CompositeDevice.hpp"
#include "rw/proximity/CollisionDetector.hpp"
#include "rw/models/WorkCell.hpp"

class QConstraint : public rw::pathplanning::QConstraint {
public:
    QConstraint();

    bool doInCollision(const rw::math::Q &q) const override;

    void doSetLog(rw::common::Log::Ptr log) override;

    virtual void doUpdate(const rw::kinematics::State &state);

    std::vector<rw::models::Device::Ptr> _devs;
    rw::models::Device::Ptr _device;
    rw::kinematics::State _state;
    rw::proximity::CollisionDetector::Ptr _detector;
    rw::models::WorkCell::Ptr _workCell;
};


#endif //PUPPY_MYQCONSTRAINT_H
