#include "state.h"

state::state()
{
    rotation = Eigen::Quaterniond::Identity();
    translation = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    ba = Eigen::Vector3d::Zero();
    bg = Eigen::Vector3d::Zero();
}

state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
        const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
{

}

state::state(const state* state_temp, bool copy)
{
    rotation = state_temp->rotation;
    translation = state_temp->translation;
    velocity = state_temp->velocity;
    ba = state_temp->ba;
    bg = state_temp->bg;
}

void state::release()
{

}