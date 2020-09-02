
//
// Created by cda on 2020/8/18.
//




#ifndef UNTITLED_DYNAMICS_H
#define UNTITLED_DYNAMICS_H





// #include "inertia_properties.h"
// #include "forward_dynamics.h"
// #include "inverse_dynamics.h"
// #include "joint_data_map.h"
// #include "link_data_map.h"
// #include "transforms.h"
// #include "declarations.h"
// #include "jsim.h"
// #include "default_dynparams_getter.h"
// #include "inertia_properties.h"
// #include "jacobians.h"
// #include "miscellaneous.h"
// #include "dynamics_parameters.h"
// #include "kinematics_parameters.h"
// #include "traits.h"




#include <Eigen/Core>
#include <Eigen/StdVector>
#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"
// define namespace and base
#define ROBCOGEN_NS single_leg
#define TARGET_NS single_leg
// define the links
#define CT_BASE fr_torso
#define CT_L0 fr_upper_leg
#define CT_L1 fr_lower_leg

// define single end effector (could also be multiple)
#define CT_N_EE 1
#define CT_EE0 fr_foot
#define CT_EE0_IS_ON_LINK 2
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 1

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
#endif //UNTITLED_DYNAMICS_H
