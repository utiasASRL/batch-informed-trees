/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez */
/* Edited by: Jonathan Gammell (Informed sampling) */

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/samplers/InformedStateSamplers.h"
#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <boost/math/constants/constants.hpp>
//For pre C++ 11 gamma function
#include <boost/math/special_functions/gamma.hpp>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) :
    base::Planner(si, "RRTstar"),
    goalBias_(0.05),
    maxDistance_(0.0),
    useKNearest_(true),
    rewireFactor_(1.1),
    k_rrg_(0u),
    r_rrg_(0.0),
    delayCC_(true),
    lastGoalMotion_(NULL),
    prune_(false),
    pruneStatesThreshold_(0.95),
    useInformedSampling_(false),
    numVerticesWorseThanSoln_(0u),
    iterations_(0u),
    collisionChecks_(0u),
    bestCost_(std::numeric_limits<double>::quiet_NaN())
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &RRTstar::setRewireFactor, &RRTstar::getRewireFactor, "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTstar::setKNearest, &RRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("prune", this, &RRTstar::setPrune, &RRTstar::getPrune, "0,1");
    Planner::declareParam<double>("prune_states_threshold", this, &RRTstar::setPruneStatesImprovementThreshold, &RRTstar::getPruneStatesImprovementThreshold, "0.:.01:1.");
    Planner::declareParam<bool>("use_informed_sampling", this, &RRTstar::setInformedSampling, &RRTstar::getInformedSampling, "0,1");

    addPlannerProgressProperty("iterations INTEGER",
                               boost::bind(&RRTstar::getIterationCount, this));
    addPlannerProgressProperty("collision checks INTEGER",
                               boost::bind(&RRTstar::getCollisionCheckCount, this));
    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&RRTstar::getBestCost, this));
}

ompl::geometric::RRTstar::~RRTstar()
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
            opt_.reset(new base::PathLengthOptimizationObjective(si_));
            //Store it back into the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    //Allocate a sampler.
    allocSampler();

    //Calculate some constants:
    double dimDbl        = (double)si_->getStateDimension();

    // k_rrg > e+e/d.  K-nearest RRT*
    k_rrg_ = rewireFactor_*(boost::math::constants::e<double>() + (boost::math::constants::e<double>() / dimDbl));

    // r_rrg > 2*(1+1/d)^(1/d)*(measure/ballvolume)^(1/d)
    r_rrg_ = rewireFactor_*2.0*std::pow((1.0 + 1.0/dimDbl)*(si_->getMeasure()/ProlateHyperspheroid::unitNBallMeasure(si_->getStateDimension())), 1.0/dimDbl);
}

void ompl::geometric::RRTstar::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    lastGoalMotion_ = NULL;
    goalMotions_.clear();

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    bool symCost = opt_->isSymmetric();

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->cost = opt_->identityCost();
        nn_->add(motion);
        startMotion_ = motion;
    }

    if (nn_->size() - numVerticesWorseThanSoln_ == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size() - numVerticesWorseThanSoln_);

    if (prune_ && !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: tree pruning was activated but since the state space %s is not a metric space, "
                  "the optimization objective might not satisfy the triangle inequality. You may need to disable pruning."
                  , getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *solution       = lastGoalMotion_;

    // \TODO Make this variable unnecessary, or at least have it
    // persist across solve runs
    base::Cost bestCost    = opt_->infiniteCost();

    bestCost_ = opt_->infiniteCost();

    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    std::vector<Motion*>       nbh;

    std::vector<base::Cost>    costs;
    std::vector<base::Cost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;

    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    if (solution)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(), solution->cost.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg_ * log((double)(nn_->size() - numVerticesWorseThanSoln_ + 1))));
    else
        OMPL_INFORM("%s: Initial rewiring radius of %.2f", getName().c_str(), std::min(maxDistance_, r_rrg_*std::pow(log((double)(nn_->size() - numVerticesWorseThanSoln_ + 1))/((double)(nn_->size() - numVerticesWorseThanSoln_ + 1)), 1/(double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while (ptc == false)
    {
        iterations_++;

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && sampler_->rng().uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            sampler_->sampleUniform(rstate);

            if (prune_ && opt_->isCostBetterThan(bestCost_, costToGo(rmotion)))
                continue;
        }

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        ++collisionChecks_;
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            rewireTest += nbh.size();
            statesGenerated++;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(),
                          compareFn);

                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + nbh.size();
                     ++i)
                {
                    if (nbh[*i] != nmotion)
                        // If nbh[*i] != nmotion, we need to perform a collision check. Technically this happens either way, but we'll give the benefit of the doubt.
                        ++collisionChecks_;
                    if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
                    {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                            valid[*i] = 1;
                        break;
                    }
                    else valid[*i] = -1;
                }
            }
            else // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            ++collisionChecks_;
                            if (si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                    valid[i] = 1;
                            }
                            else valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                            valid[i] = 1;
                    }
                }
            }

            if (prune_)
            {
                if (opt_->isCostBetterThan(costToGo(motion, false), bestCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                }
                else // If the new motion does not improve the best cost it is ignored.
                {
                    --statesGenerated;
                    si_->freeState(motion->state);
                    delete motion;
                    continue;
                }
            }
            else
            {
            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);
            }

            bool checkForSolution = false;
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                            if (valid[i] == 0)
                            {
                                ++collisionChecks_;
                                motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                            }
                            else
                                motionValid = (valid[i] == 1);

                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[i]);

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            checkForSolution = true;
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                for (size_t i = 0; i < goalMotions_.size(); ++i)
                {
                    if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                    {
                        if (std::isfinite(bestCost.value()) == false)
                        {
                            OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u vertices)", getName().c_str(), goalMotions_[i]->cost, iterations_, nn_->size() - numVerticesWorseThanSoln_);
                        }
                        bestCost = goalMotions_[i]->cost;
                        bestCost_ = bestCost;
                        updatedSolution = true;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
                    if (sufficientlyShort)
                    {
                        solution = goalMotions_[i];
                        break;
                    }
                    else if (!solution ||
                             opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
                    {
                        solution = goalMotions_[i];
                        updatedSolution = true;
                    }
                }

                if (updatedSolution)
                {
                    if (useInformedSampling_ == true)
                    {
                        countNumberOfVerticesWorseThanSoln();
                        if (useKNearest_ == false)
                        {
                            r_rrg_ = rewireFactor_*2.0*std::pow((1.0 + 1.0/(double)(si_->getStateDimension()))*(sampler_->as<ompl::base::InformedStateSampler>()->getInformedMeasure()/ProlateHyperspheroid::unitNBallMeasure(si_->getStateDimension())), 1.0/(double)(si_->getStateDimension()));
                        }
                    }

                    if (prune_)
                    {
                        int n = pruneTree(bestCost_);
                        statesGenerated -= n;
                    }

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution = solution->parent; // Do not include goal state to simplify code.

                        do
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        } while (intermediate_solution->parent != 0); // Do not include the start state.

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
            {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == NULL);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != NULL)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *geoPath = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            geoPath->append(mpath[i]->state);

        base::PathPtr path(geoPath);
        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        if (approximate)
            psol.setApproximate(approximatedist);
        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, bestCost, sufficientlyShort);
        pdef_->addSolutionPath(psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost %.3f", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_);

    return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstar::getNeighbors(Motion *motion, std::vector<Motion*> &nbh)
{
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrg_ * log((double)(nn_->size() - numVerticesWorseThanSoln_ + 1)));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(maxDistance_, r_rrg_*std::pow(log((double)(nn_->size() - numVerticesWorseThanSoln_ + 1))/((double)(nn_->size() - numVerticesWorseThanSoln_ + 1)), 1/(double)(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}


void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    for (std::vector<Motion*>::iterator it = m->parent->children.begin ();
        it != m->parent->children.end (); ++it)
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
    data.properties["collision_checks INTEGER"] =
        boost::lexical_cast<std::string>(collisionChecks_);
}

int ompl::geometric::RRTstar::pruneTree(const base::Cost pruneTreeCost)
{
    const int tree_size = nn_->size();
    pruneScratchSpace_.newTree.reserve(tree_size);
    pruneScratchSpace_.newTree.clear();
    pruneScratchSpace_.toBePruned.reserve(tree_size);
    pruneScratchSpace_.toBePruned.clear();
    pruneScratchSpace_.candidates.clear();
    pruneScratchSpace_.candidates.push_back(startMotion_);
    std::size_t j = 0;
    while (j != pruneScratchSpace_.candidates.size())
    {
        Motion *candidate = pruneScratchSpace_.candidates[j++];
        if (opt_->isCostBetterThan(pruneTreeCost, costToGo(candidate)))
            pruneScratchSpace_.toBePruned.push_back(candidate);
        else
        {
            pruneScratchSpace_.newTree.push_back(candidate);
            pruneScratchSpace_.candidates.insert(pruneScratchSpace_.candidates.end(),
                candidate->children.begin(), candidate->children.end());
}
    }

    // To create the new nn takes one order of magnitude in time more than just checking how many
    // states would be pruned. Therefore, only prune if it removes a significant amount of states.
    if ((double)pruneScratchSpace_.newTree.size() / tree_size < pruneStatesThreshold_)
{
        for (std::size_t i = 0; i < pruneScratchSpace_.toBePruned.size(); ++i)
            deleteBranch(pruneScratchSpace_.toBePruned[i]);

        nn_->clear();
        nn_->add(pruneScratchSpace_.newTree);

        return (tree_size - pruneScratchSpace_.newTree.size());
}
    return 0;
}

void ompl::geometric::RRTstar::deleteBranch(Motion *motion)
{
    removeFromParent(motion);

    std::vector<Motion *>& toDelete = pruneScratchSpace_.candidates;
    toDelete.clear();
    toDelete.push_back(motion);

    while (!toDelete.empty())
    {
        Motion *mto_delete = toDelete.back();
        toDelete.pop_back();

        for(std::size_t i = 0; i < mto_delete->children.size(); ++i)
            toDelete.push_back(mto_delete->children[i]);

        si_->freeState(mto_delete->state);
        delete mto_delete;
    }
}

ompl::base::Cost ompl::geometric::RRTstar::costToGo(const Motion *motion, const bool shortest) const
{
    base::Cost costToCome;
    if (shortest)
        costToCome = opt_->motionCost(startMotion_->state, motion->state); // h_s
    else
        costToCome = motion->cost; //d_s

    const base::Cost costToGo = base::goalRegionCostToGo(motion->state, pdef_->getGoal().get()); // h_g
    return opt_->combineCosts(costToCome, costToGo); // h_s + h_g
}

void ompl::geometric::RRTstar::countNumberOfVerticesWorseThanSoln()
{
    //Variable
    //The list of data in the nearest neighbours struct
    std::vector<Motion*> data;

    //Get the list
    nn_->list(data);


    //Reset the counter:
    numVerticesWorseThanSoln_ = 0u;
    for (unsigned int i = 0u; i < data.size(); ++i)
    {
        //Variable
        //The heuristic estimate of a solution constrained to go through the state
        double heuristicValue;

        //Calculate the heuristic value:
        heuristicValue = boost::static_pointer_cast<base::InformedStateSampler>(sampler_)->getHeuristicValue(data.at(i)->state);

        //Check and remove if necessary
        if (heuristicValue > bestCost_.value())
        {
            //Increment the counter:
            ++numVerticesWorseThanSoln_;
        }
        //No else
    }
}

void ompl::geometric::RRTstar::setInformedSampling(bool informedSampling)
{
    //Store the setting. We only want to create a new sampler if it has changed, but we do it in 2 if-steps so that we can also check if we have a planner yet...
    if (informedSampling != useInformedSampling_)
    {
        useInformedSampling_ = informedSampling;

        //If we currently have a planner, we need to make a new one
        if(sampler_)
        {
            //Reset the sampler
            sampler_.reset();

            //Create the sampler
            allocSampler();
        }
    }
}


void ompl::geometric::RRTstar::allocSampler()
{
    if (useInformedSampling_)
    {
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        sampler_ = opt_->allocInformedStateSampler(si_->getStateSpace().get(), pdef_, &bestCost_);
    }
    else
    {
        sampler_ = si_->allocStateSampler();
    }
}
