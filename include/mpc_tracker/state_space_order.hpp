#pragma once

/**
 * @brief Robot input state
 *
 */
namespace MPC_INPUT
{
static constexpr int ANGULAR_VEL_YAW = 0;
static constexpr int TWIST_X         = 1;
static constexpr int DIM             = 2;
}  // namespace MPC_INPUT

/**
 * @brief Robot state-space order in frenet-serret coordinate which is used in MPC
 *
 */
namespace MPC_STATE_SPACE
{
static constexpr int X_F   = 0;
static constexpr int Y_F   = 1;
static constexpr int YAW_F = 2;
static constexpr int DIM   = 3;
}  // namespace MPC_STATE_SPACE
