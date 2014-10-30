/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#ifndef OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
#define OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_

//We inherit from StateSampler
#include "ompl/base/StateSampler.h"
//Deriving functions must be able to sample within a given cost
#include "ompl/base/Cost.h"
//We use a pointer to the problem definition to access problem and solution data.
#include "ompl/base/ProblemDefinition.h"
//The goal definitions
#include "ompl/base/Goal.h"

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(InformedStateSampler);

        /** \brief An abstract class for state samplers that use information
        about the current solution to limit future search to a planning
        subproblem that contains all possibly better solutions. */
        class InformedStateSampler : public StateSampler
        {
        public:

            /** \brief Construct a sampler for \e space that generates states by sampling a \e heuristic estimate of the cost function between a \e start and \e goal state. */
            InformedStateSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost);
            virtual ~InformedStateSampler(void)
            {
            }

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is less than the current best cost (as defined by the pointer passed at construction). By default dereferences the pointer and calls the sampleUniform(State*, Cost) of the deriving class*/
            virtual void sampleUniform(State* state);

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is less than the provided cost */
            virtual void sampleUniform(State* state, const Cost& maxCost) = 0;

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is between the provided costs*/
            virtual void sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost) = 0;

            /** \brief Calculate the heuristic value for a given state */
            virtual double getHeuristicValue(const State* state) = 0;

            /** \brief The measure of the space being searched informed by the knowledge of the problem and the solution, i.e., include consideration of the boundary. Should return the measure of the entire space if no solution has been found */
            virtual double getInformedMeasure() const = 0;

            /** \brief The measure of the informed space if the best cost was as given. Should be able to be calculated regardless of whether there's a solution or not and should \e ignore the existence of a boundary*/
            virtual double getHypotheticalMeasure(const Cost& hypCost) const = 0;

            /** \brief sampleUniformNear is currently not supported for InformedStateSamplers*/
            virtual void sampleUniformNear(State* state, const State* near, const double distance);

            /** \brief sampleGaussian is currently not supported for InformedStateSamplers*/
            virtual void sampleGaussian(State* state, const State* mean, const double stdDev);


        protected:
            /** A copy of the problem definition */
            ProblemDefinitionPtr probDefn_;
            /** A pointer to the best cost found so far, the mechanism through which the sampler is "informed". */
            const Cost* bestCostPtr_;

        private:
        };



        /** \brief A default rejection sampling scheme that samples uniformly from the entire planning domain.
        Samples are rejected until one is found that has a heuristic solution estimate that is less than the current solution.
        In general, direct sampling of the informed subset is much better, but this is a general default.
        This is \e completely \e untested.

        @par TODO
        - Test.
        */
        class RejectionSampler : public InformedStateSampler
        {
        public:

            /** \brief Construct the sampler with a pointer to the planner so it can extract solution information. */
            RejectionSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost);
            virtual ~RejectionSampler()
            {
            }

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is less than the provided cost */
            void sampleUniform(State* state, const Cost& maxCost);

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is between the provided costs*/
            void sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost);

            /** \brief Calculate the heuristic value for a given state */
            double getHeuristicValue(const State* state);

            /** \brief The measure of the space being searched informed by the knowledge of the problem and the solution. As rejection sampling has no closed-form knowledge of the informed subset, the measure of the informed space is always the measure of the entire space. */
            double getInformedMeasure() const;

            /** \brief The measure of the informed space if the best cost was as given. As rejection sampling has no closed-form knowledge of the informed subset, the measure of the informed space will always be the measure of the entire space, regardless of the current solution. */
            double getHypotheticalMeasure(const Cost& /*hypCost*/) const;

            /** \brief Set the seed of all the state samplers. */
            void setLocalSeed(boost::uint32_t localSeed);

        private:
            //Variables
            /** \brief The basic raw sampler used to generate samples to keep/reject. */
            StateSamplerPtr baseSampler_;
            //The start and goal states
            State* startState_;
            GoalPtr goal_;
        };



        /** \brief An informed sampler for problems seeking to minimize path length.

        It focuses the search to the subset of a problem that can improve a current solution, which is a prolate hyperspheroid (PHS)
        (a special type of an hyperellipsoid) and can be sampled directly.
        Doing so considers all homotopy classes that can provide a better solution while guaranteeing a non-zero probability
        of improving a solution regardless of the size of the planning domain, the number of state dimensions, and how close
        the current solution is to the theoretical minimum.
        Currently only implemented for problems with a single goal in R^n (i.e., RealVectorStateSpace), SE(2) (i.e., SE2StateSpace), and SE(3) (i.e., SE3StateSpace).
        Until an initial solution is found, this sampler simply passes-through to a uniform distribution over the entire state space.
        @par J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Informed RRT*: Optimal Sampling-based
        Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic."
        IROS 2014. <a href="http://arxiv.org/abs/1404.2334">arXiv:1404.2334 [cs.RO]</a>.
        <a href="http://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
        <a href="http://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>.

        @par TODO
        - Use OptimizationObjective properly.
        - Handle compound spaces more gracefully.
        - Handle other types of goals? */
        class PathLengthInformedSampler : public InformedStateSampler
        {
        public:

            /** \brief Construct the sampler with a pointer to the planner so it can extract solution information. */
            PathLengthInformedSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost);

            virtual ~PathLengthInformedSampler();

            /** \brief Draw a sample uniformly distributed over the subsetof the planning problem that can improve the current solution. */
            void sampleUniform(State* statePtr, const Cost& maxCost);

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is between the provided costs*/
            void sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost);

            /** \brief Calculate the heuristic value for a given state */
            double getHeuristicValue(const State* statePtr);

            /** \brief The measure of the space being searched informed by the knowledge of the problem and the solution, i.e., include consideration of the boundary. */
            double getInformedMeasure() const;

            /** \brief The measure of the informed space if the best cost was as given. Should be able to be calculated regardless of whether there's a solution or not and should \e ignore the existence of a boundary*/
            double getHypotheticalMeasure(const Cost& hypCost) const;

            /** \brief Set the seed of all the state samplers. */
            void setLocalSeed(boost::uint32_t localSeed);

            RNG& rng()
            {
                return baseSampler_->rng();
            }

        protected:

        private:
            //Some functions that used to be public by are still helpful to have around:
            /** \brief Draw a sample uniformly distributed over the subsetof the planning problem that can improve the current solution, ignoring the bounds of the state space. */
            void sampleUniformIgnoreBounds(State* statePtr, const Cost& maxCost);

            /** \brief Sample uniformly in the subset of the state space whose heuristic estimate is between the provided costs, ignoring the bounds of the state space*/
            void sampleUniformIgnoreBounds(State* statePtr, const Cost& minCost, const Cost& maxCost);

            //Variables
            /** \brief The prolate hyperspheroid description of the sub problem */
            ompl::ProlateHyperspheroidPtr phsPtr_;

            /** \brief The state space of the planning problem that is informed by the heuristics, i.e., in SE(2), R^2*/
            const StateSpace* informedSubSpace_;

            /** \brief The state space of the planning problem that is \e not informed by the heuristics, i.e., in SE(2), SO(2)*/
            const StateSpace* uninformedSubSpace_;

            /** \brief The sampler to use in cases where informed sampling cannot be used. I.e., Before a solution is found, or if the solution does not reduce the search space. */
            StateSamplerPtr baseSampler_;

            /** \brief The sampler to use on the uninformed subspace. */
            StateSamplerPtr uninformedSubSampler_;

            /// @cond IGNORE
            //The indices for compounding state spaces (i.e., SE2 and SE3)
            static const unsigned int INFORMED_IDX = 0;
            static const unsigned int UNINFORMED_IDX = 1;
            /// @endcond


        }; //PathLengthInformedSampler
    }
}


#endif //OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
