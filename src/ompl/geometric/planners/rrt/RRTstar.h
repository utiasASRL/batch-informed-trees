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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan */
/* Edited by: Jonathan Gammell (Informed sampling) */

#ifndef OMPL_CONTRIB_RRT_STAR_RRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_RRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <limits>
#include <vector>
#include <utility>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTstar
           @par Short description
           \ref gRRTstar "RRT*" (optimal RRT) is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. \ref gRRTstar "RRT*" algorithm is
           guaranteed to converge to an optimal solution, while its
           running time is guaranteed to be a constant factor of the
           running time of the \ref gRRT "RRT". The notion of optimality is with
           respect to the distance function defined on the state space
           we are operating on. See ompl::base::Goal::setMaximumPathLength() for
           how to set the maximally allowed path length to reach the goal.
           If a solution path that is shorter than ompl::base::Goal::getMaximumPathLength() is
           found, the algorithm terminates before the elapsed time.
           @par External documentation
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research (to appear), 2011.
           <a href="http://arxiv.org/abs/1105.1186">http://arxiv.org/abs/1105.1186</a>
        */

        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRTstar : public base::Planner
        {
        public:

            RRTstar(const base::SpaceInformationPtr &si);

            virtual ~RRTstar();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*) */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Motion*>());
            }

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collsion checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Use \e Informed \e RRT*.
            This means that once a problem is found, the search is focused only to the subproblem that could contain a better solution.
            Currently only implemented for problems with a single goal that are seeking to minimize path length in R^n (i.e., RealVectorStateSpace), SE(2) (i.e., SE2StateSpace), or SE(3) (i.e., SE3StateSpace).
            @par J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Informed RRT*: Optimal Sampling-based
            Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic."
            IROS 2014. <a href="http://arxiv.org/abs/1404.2334">arXiv:1404.2334 [cs.RO]</a>.
            <a href="http://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
            <a href="http://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state of informed sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            virtual void setup();

            /** \brief Get the seed for the underlying RNG and StateSampler. Useful for running different settings with the exact same pseudorandom sequence. */
            boost::uint32_t getLocalSeed() const
            {
                return sampler_->getLocalSeed();
            }

            /** \brief Set the seed for the underlying and StateSampler. Useful for running different settings with the exact same pseudorandom sequence. */
            void setLocalSeed(boost::uint32_t seed)
            {
                sampler_->setLocalSeed(seed);
            }

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const;

            std::string getCollisionCheckCount() const;

            std::string getBestCost() const;
            ///////////////////////////////////////

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()),
                    parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief The cost up to this motion */
                base::Cost        cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
                base::Cost        incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief Create the sampler */
            void allocSampler();

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost>& costs,
                                 const base::OptimizationObjective &opt) :
                    costs_(costs), opt_(opt)
                {}
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i],costs_[j]);
                }
                const std::vector<base::Cost>& costs_;
                const base::OptimizationObjective &opt_;
            };

            enum DistanceDirection { FROM_NEIGHBORS, TO_NEIGHBORS };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                switch (distanceDirection_)
                {
                case FROM_NEIGHBORS:
                    return si_->distance(a->state, b->state);
                case TO_NEIGHBORS:
                    return si_->distance(b->state, a->state);
                }
                return 0; // remove warning
            }

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(Motion *motion, std::vector<Motion*> &nbh);

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief Pretend to prune the graph of any vertices that have a heuristic value (lower-bounding solution cost constrained to pass through the vertex) that is greater than the current solution. Given the current memory model, for now this actually just calculates how many vertices \e would be pruned and stores that number in numPrunedVertices_*/
            void fakeHeuristicGraphPruning();

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times k_rrg* > k_rrg*) */
            double                                         rewireFactor_;

            /** \brief Option to delay and reduce collision checking within iterations */
            bool                                           delayCC_;

            /** \brief Option to use k-nearest search for rewiring */
            bool                                           useKNearest_;

            /** \brief Option to use informed sampling */
            bool                                           useInformedSampling_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr                 opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion*>                           goalMotions_;

            /** \brief A constant for k-nearest rewiring calculations */
            double                                         k_rrg_;
            /** \brief A constant for r-disc rewiring calculations */
            double                                         r_rrg_;

            /** \brief The number of vertices in the graph that are considered "pruned" */
            unsigned int                                   numPrunedVertices_;

            //////////////////////////////
            // Planner progress properties

            /** \brief Number of iterations the algorithm performed */
            unsigned int                                   iterations_;

            /** \brief Number of collisions checks performed by the algorithm */
            unsigned int                                   collisionChecks_;

            /** \brief Best cost found so far by algorithm */
            base::Cost                                     bestCost_;

            /** \brief Directionality of distance computation for
                nearest neighbors. Either from neighbors to new state,
                or from new state to neighbors. */
            DistanceDirection                              distanceDirection_;
        };

    }
}

#endif
