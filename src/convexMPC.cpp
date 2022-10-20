#include "quadruped_control_library/convexMPC.hpp"
#include <iostream>

ConvexMPC::ConvexMPC(uint horizon, double dtMPC, double gravity, uint numLegs) {
  horizon_ = horizon;
  dtMPC_ = dtMPC;
  gravity_ = gravity;
  numLegs_ = numLegs;
}

void ConvexMPC::updateStateWeights(Eigen::Matrix<double, 13, 1> stateWeights) {
  stateWeights_.resize(13 * horizon_, 13 * horizon_);
  stateWeights_.setIdentity();
  stateWeights_.diagonal() = stateWeights.replicate(horizon_, 1);
}

void ConvexMPC::updateForceWeights(double forceWeights) {
  forceWeights_.resize(3 * numLegs_ * horizon_, 3 * numLegs_ * horizon_);
  forceWeights_.setIdentity();
  forceWeights_ = forceWeights_ * forceWeights;
}

std::vector<Eigen::Vector3d>
ConvexMPC::solveMPC(const MPCdata &currentState, const MPCdata &desiredState,
                    const std::vector<bool> &gaitTable,
                    const Eigen::Matrix<double, 3, -1> &rFeet) {
  Eigen::Matrix<double, 13, 1> x0;
  x0 << currentState.rotation[0], currentState.rotation[1],
      currentState.rotation[2], currentState.position[0],
      currentState.position[1], currentState.position[2],
      currentState.rotation_velocity[0], currentState.rotation_velocity[1],
      currentState.rotation_velocity[2], currentState.linear_velocity[0],
      currentState.linear_velocity[1], currentState.linear_velocity[2],
      gravity_;
  double yc = std::cos(currentState.rotation[2]);
  double ys = std::sin(currentState.rotation[2]);
  Eigen::Matrix<double, 3, 3> RYaw;
  RYaw << yc, -ys, 0, ys, yc, 0, 0, 0, 1;
  auto [reductionVector, numLegsActive] = getGaitData(gaitTable);
  auto Adt = getAdt(RYaw);
  auto Bdt = getBdt(RYaw, rFeet);
  auto [Aqp, Bqp] = getAqp_Bqp(Adt, Bdt);
  auto xD = getTrajectory(desiredState);
  auto [H, g] = getH_g(x0, xD, Aqp, Bqp);
  auto [HRed, gRed] = getHRed_gRed(H, g, reductionVector, numLegsActive);
  auto lB = getLowerBoundary(numLegsActive);
  auto uB = getUpperBoundary(numLegsActive, maxForce_);
  auto A = getConstraintMatrix(numLegsActive, mu_);
  Eigen::Matrix<double, -1, 1> mpcResult;
  mpcResult.resize(numLegsActive * 3, 1);
  qpOASES::QProblem problem_red(numLegsActive * 3, numLegsActive * 5);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem_red.setOptions(op);
  qpOASES::int_t nWSR = 100;
  problem_red.init(HRed.data(), gRed.data(), A.data(), NULL, NULL, lB.data(),
                   uB.data(), nWSR);
  problem_red.getPrimalSolution(mpcResult.data());
  std::vector<Eigen::Vector3d> mpcForces;
  for (int i = 0, j = 0; i < numLegs_; i++) {
    if (reductionVector[i * 3]) {
      mpcForces.push_back(
          Eigen::Vector3d(mpcResult[j], mpcResult[j + 1], mpcResult[j + 2]));
      j += 3;
    } else {
      mpcForces.push_back(Eigen::Vector3d(0, 0, 0));
    }
  }
  return mpcForces;
}

Eigen::Matrix<double, -1, 1>
ConvexMPC::getTrajectory(const MPCdata &desiredState) {
  Eigen::Matrix<double, -1, 1> trajectory;
  trajectory.resize(13 * horizon_);
  Eigen::Matrix<double, 13, 1> trajectoryInitial;
  // set roll,pitch,vertical velocity and gravity to zero (useless components)
  trajectoryInitial << desiredState.rotation[0], desiredState.rotation[1],
      desiredState.rotation[2], desiredState.position[0],
      desiredState.position[1], desiredState.position[2], 0.0, 0.0,
      desiredState.rotation_velocity[2], desiredState.linear_velocity[0],
      desiredState.linear_velocity[1], 0.0, 0;
  for (int i = 0; i < horizon_; i++) {
    trajectory.block(i * 13, 0, 13, 1) = trajectoryInitial;
    trajectory(i * 13 + 2, 0) = desiredState.rotation[2] +
                                i * dtMPC_ * desiredState.rotation_velocity[2];
    trajectory(i * 13 + 3, 0) =
        desiredState.position[0] + i * dtMPC_ * desiredState.linear_velocity[0];
    trajectory(i * 13 + 4, 0) =
        desiredState.position[1] + i * dtMPC_ * desiredState.linear_velocity[1];
  }
  return trajectory;
}

std::tuple<std::vector<bool>, uint>
ConvexMPC::getGaitData(const std::vector<bool> &gaitTable) {
  uint numLegsActive = std::count(gaitTable.begin(), gaitTable.end(), 1);
  std::vector<bool> reductionVector;
  for (int i = 0; i < horizon_ * numLegs_; i++) {
    if (gaitTable[i] == 1) {
      reductionVector.insert(reductionVector.end(), 3, 1);
    } else {
      reductionVector.insert(reductionVector.end(), 3, 0);
    }
  }
  return {reductionVector, numLegsActive};
}

Eigen::Matrix<double, -1, 1> ConvexMPC::getLowerBoundary(uint numLegsActive) {
  return Eigen::Matrix<double, -1, 1>::Constant(numLegsActive * 5, 0.0);
}

Eigen::Matrix<double, -1, 1> ConvexMPC::getUpperBoundary(uint numLegsActive,
                                                         double maxForce) {
  Eigen::Matrix<double, -1, 1> upperBoundary(numLegsActive * 5, 1);
  for (int i = 0; i < numLegsActive; i++) {
    upperBoundary(i * 5 + 0, 0) = 5e10;
    upperBoundary(i * 5 + 1, 0) = 5e10;
    upperBoundary(i * 5 + 2, 0) = 5e10;
    upperBoundary(i * 5 + 3, 0) = 5e10;
    upperBoundary(i * 5 + 4, 0) = maxForce;
  }
  return upperBoundary;
}

Eigen::Matrix<double, -1, -1, Eigen::RowMajor>
ConvexMPC::getConstraintMatrix(uint numLegsActive, double mu) {
  Eigen::Matrix<double, 5, 3> constraintBlock;
  double muInv = 1.0 / mu;
  constraintBlock << muInv, 0.0, 1.0, -muInv, 0.0, 1.0, 0.0, muInv, 1.0, 0.0,
      -muInv, 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> constraintMatrix(
      numLegsActive * 5, numLegsActive * 3);
  constraintMatrix.setZero();
  for (int i = 0; i < numLegsActive; i++) {
    constraintMatrix.block(i * 5, i * 3, 5, 3) = constraintBlock;
  }
  return constraintMatrix;
}

Eigen::Matrix<double, 13, 13>
ConvexMPC::getAdt(const Eigen::Matrix<double, 3, 3> &rYaw) {
  Eigen::Matrix<double, 13, 13> Adt;
  Adt.setZero();
  Adt.setIdentity();
  Adt.block(0, 6, 3, 3) = rYaw.transpose() * dtMPC_;
  Adt.block(3, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dtMPC_;
  Adt(11, 12) = dtMPC_;
  return Adt;
}

Eigen::Matrix<double, 13, -1>
ConvexMPC::getBdt(const Eigen::Matrix<double, 3, 3> &rYaw,
                  const Eigen::Matrix<double, 3, -1> &rFeet) {
  Eigen::Matrix<double, 13, -1> Bdt;
  Bdt.resize(13, 3 * numLegs_);
  Bdt.setZero();
  Eigen::Matrix<double, 3, 3> IWorld_inv, IWorld;
  IWorld = rYaw * IBody_ * rYaw.transpose();
  IWorld_inv = IWorld.inverse();
  for (int i = 0; i < numLegs_; i++) {
    Eigen::Matrix<double, 3, 3> crossMat;
    crossMat << 0.0, -rFeet(2, i), rFeet(1, i), rFeet(2, i), 0.0, -rFeet(0, i),
        -rFeet(1, i), rFeet(0, i), 0.0;
    Bdt.block(6, i * 3, 3, 3) = IWorld_inv * crossMat * dtMPC_;
    Bdt.block(9, i * 3, 3, 3) =
        Eigen::Matrix<double, 3, 3>::Identity() * dtMPC_ / mass_;
  }
  return Bdt;
}

std::tuple<Eigen::Matrix<double, -1, 13>, Eigen::Matrix<double, -1, -1>>
ConvexMPC::getAqp_Bqp(const Eigen::Matrix<double, 13, 13> &Adt,
                      Eigen::Matrix<double, 13, -1> &Bdt) {
  Eigen::Matrix<double, 13, 13> powerMats[20];
  Eigen::Matrix<double, -1, 13> Aqp;
  Eigen::Matrix<double, -1, -1> Bqp;
  Aqp.resize(13 * horizon_, 13);
  Aqp.setZero();
  Bqp.resize(13 * horizon_, numLegs_ * 3 * horizon_);
  Bqp.setZero();
  powerMats[0].setIdentity();
  for (int i = 1; i < horizon_ + 1; i++) {
    powerMats[i] = Adt * powerMats[i - 1];
  }
  for (int r = 0; r < horizon_; r++) {
    Aqp.block(13 * r, 0, 13, 13) = powerMats[r + 1];
    for (int c = 0; c < horizon_; c++) {
      if (r >= c) {
        int a_num = r - c;
        Bqp.block(13 * r, numLegs_ * 3 * c, 13, numLegs_ * 3) =
            powerMats[a_num] * Bdt;
      }
    }
  }
  return {Aqp, Bqp};
}

std::tuple<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, 1>>
ConvexMPC::getH_g(const Eigen::Matrix<double, 13, 1> &x0,
                  const Eigen::Matrix<double, -1, 1> &xD,
                  const Eigen::Matrix<double, -1, 13> &Aqp,
                  const Eigen::Matrix<double, -1, -1> &Bqp) {
  Eigen::Matrix<double, -1, -1> qH;
  Eigen::Matrix<double, -1, 1> qG;
  qH.resize(3 * numLegs_ * horizon_, 3 * numLegs_ * horizon_);
  qG.resize(3 * numLegs_ * horizon_, 1);
  //          120x130           130x130    130x120   120x120
  qH = 2 * (Bqp.transpose() * stateWeights_ * Bqp + forceWeights_);
  qG = 2 * Bqp.transpose() * stateWeights_ * (Aqp * x0 - xD);
  return {qH, qG};
}

std::tuple<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>,
           Eigen::Matrix<double, -1, 1>>
ConvexMPC::getHRed_gRed(const Eigen::Matrix<double, -1, -1> &H,
                        const Eigen::Matrix<double, -1, 1> &g,
                        std::vector<bool> reductionVector, uint numLegsActive) {
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> HRed;
  HRed.resize(numLegsActive * 3, numLegsActive * 3);
  Eigen::Matrix<double, -1, 1> gRed;
  gRed.resize(numLegsActive * 3, 1);
  for (int i = 0, k = 0; i < numLegsActive * 3; i++) {
    while (reductionVector[k] == 0) {
      k++;
    }
    for (int j = 0, l = 0; j < numLegsActive * 3; j++) {
      while (reductionVector[l] == 0) {
        l++;
      }
      HRed(i, j) = H(k, l);
      l++;
    }
    gRed(i, 0) = g(k, 0);
    k++;
  }
  return {HRed, gRed};
}