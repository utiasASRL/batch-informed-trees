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

#include "ompl/base/samplers/InformedStateSamplers.h"
#include "ompl/util/Exception.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

//For strncmp
#include <string.h>

namespace ompl
{
    namespace base
    {
        //The base InformedStateSampler class:
        InformedStateSampler::InformedStateSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : StateSampler(space),
            probDefn_(probDefn),
            bestCostPtr_(bestCost)
        {
        }

        void InformedStateSampler::sampleUniform(State* state)
        {
            if (bestCostPtr_ == false)
            {
              throw Exception ("A valid cost pointer must be passed at construction.");
            }

            this->sampleUniform(state, *bestCostPtr_);
        }

        void InformedStateSampler::sampleUniformNear(State* state, const State* near, const double distance)
        {
            throw Exception("sampleUniformNear is not implemented for heuristic samplers.");
        }


        void InformedStateSampler::sampleGaussian(State* state, const State* mean, const double stdDev)
        {
            throw Exception("sampleUniformNear is not implemented for heuristic samplers.");
        }


        //The default rejection-sampling class:
        RejectionSampler::RejectionSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : InformedStateSampler(space, probDefn, bestCost)
        {
            //Sanity check the problem.
            if (probDefn_->getStartStateCount() != 1u)
            {
                throw Exception("Rejection sampling currently only supports 1 start state.");
            }//No else

            if (probDefn_->hasOptimizationObjective() == false)
            {
                throw Exception("No optimization objective specified during creation of the rejection sampler.");
            }//No else

            //Create the basic sampler
            baseSampler_ = space_->allocDefaultStateSampler();

            //Set it's seed to the same as mine
            baseSampler_->setLocalSeed( this->getLocalSeed() );

            //Store the start and goal
            startState_ = probDefn_->getStartState(0u);
            goal_ = probDefn_->getGoal();
        }

        void RejectionSampler::sampleUniform(State* statePtr, const Cost& maxCost)
        {
            //Sample from the entire domain until the sample has a suitable heuristic value.
            do
            {
                this->sampleUniform(statePtr, maxCost);
            }
            while ( this->getHeuristicValue(statePtr) >= maxCost.value() );
        }

        void RejectionSampler::sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            //Sample from the larger heuristic bound until the sample is greater than the smaller bound.
            do
            {
                this->sampleUniform(statePtr, maxCost);
            }
            while ( this->getHeuristicValue(statePtr) < minCost.value() );
        }

        double RejectionSampler::getHeuristicValue(const State* statePtr)
        {
            //Variable
            return probDefn_->getOptimizationObjective()->motionCostHeuristic(startState_, statePtr).value() + probDefn_->getOptimizationObjective()->costToGo(statePtr, goal_.get()).value();
        }

        double RejectionSampler::getInformedMeasure() const
        {
            return space_->getMeasure();
        }

        double RejectionSampler::getHypotheticalMeasure(const Cost& /*hypCost*/) const
        {
            return space_->getMeasure();
        }

        void RejectionSampler::setLocalSeed(boost::uint32_t localSeed)
        {
            //Set the seed of my base class, i.e., my rng_ memeber variable
            StateSampler::setLocalSeed(localSeed);

            //Set the seed for my member sub-samplers as well:
            baseSampler_->setLocalSeed( this->getLocalSeed() );
        }








        //The direct ellipsoid-sampling class for path-length:
        PathLengthInformedSampler::PathLengthInformedSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : InformedStateSampler(space, probDefn, bestCost)
        {
            //Variables
            //The foci of the ellipse as State*s
            State* start;
            State* goal;
            //The foci of the ellipse as std::vectors
            std::vector<double> xStart;
            std::vector<double> xGoal;

            //If the state space is not compound, it better be a RealVector
            if (space_->isCompound() == false && std::strncmp(space_->getName().c_str(), "RealVector", std::strlen("RealVector")) != 0 )
            {
                throw Exception("A non-compound state space that is not RealVector...");
            }
            else if (space_->isCompound() == true && std::strncmp(space_->getName().c_str(), "SE2", std::strlen("SE2")) != 0 && std::strncmp(space_->getName().c_str(), "SE3", std::strlen("SE3")) != 0)
            {
                throw Exception ("A compound state space that is not SE2 or SE3...");
            }

            //Sanity check the problem.
            if (probDefn_->getStartStateCount() != 1u)
            {
                throw Exception("The path-length sampler currently only supports 1 start state.");
            }

            if (probDefn_->getGoal()->hasType(GOAL_STATE) == false)
            {
                throw Exception("The path-length sampler currently only supports goals that can be cast to goal states.");
            }

            //Create a sampler for the whole space that we can use if we have no information
            baseSampler_ = space_->allocDefaultStateSampler();

            //Set it's seed to the same as mine
            baseSampler_->setLocalSeed( this->getLocalSeed() );

            //Check if the space is compound
            if (space_->isCompound() == false)
            {
                //Store the foci
                start = probDefn_->getStartState(0u);
                goal = probDefn_->getGoal()->as<GoalState>()->getState();

                //It is not, the informed subspace is the full space
                informedSubSpace_ = space_;

                //And the uniformed subspace and its associated sampler are null
                uninformedSubSpace_ = NULL;
                uninformedSubSampler_ = StateSamplerPtr();
            }
            else
            {
                //Sanity check that there's only 2 subspaces in the compound
                if (space_->as<CompoundStateSpace>()->getSubspaceCount() != 2)
                {
                    throw Exception("Did not expect there to be more than 2 subspaces");
                }

                //Store the foci
                start = probDefn_->getStartState(0u)->as<CompoundState>()->components[INFORMED_IDX];
                goal = probDefn_->getGoal()->as<GoalState>()->getState()->as<CompoundState>()->components[INFORMED_IDX];

                //The informed subset is the real vector space. I wish I could keep it as a shared pointer... but the whole space was a raw, so....
                informedSubSpace_ = space_->as<CompoundStateSpace>()->getSubspace(INFORMED_IDX).get();

                //And the uninformed subspace is the remainder. I wish I could keep it as a shared pointer... but the whole space was a raw, so....
                uninformedSubSpace_ = space_->as<CompoundStateSpace>()->getSubspace(UNINFORMED_IDX).get();

                //Create a sampler for the uniformed subset:
                uninformedSubSampler_ = uninformedSubSpace_->allocDefaultStateSampler();

                //Set it's seed to the same as mine
                uninformedSubSampler_->setLocalSeed( this->getLocalSeed() );
            }

            //Now extract the foci of the ellipse
            informedSubSpace_->copyToReals(xStart, start);
            informedSubSpace_->copyToReals(xGoal, goal);

            //Create the definition of the PHS
            phsPtr_ = ProlateHyperspheroidPtr(new ProlateHyperspheroid(informedSubSpace_->getDimension(), &xStart[0], &xGoal[0]));
        }

        PathLengthInformedSampler::~PathLengthInformedSampler()
        {
            //dtor
        }


        void PathLengthInformedSampler::sampleUniform(State* statePtr, const Cost& maxCost)
        {
            //Check if a solution path has been found
            if (std::isfinite(maxCost.value()) == false)
            {
                //We don't have a solution yet, we sample from our basic sampler instead...
                baseSampler_->sampleUniform(statePtr);
            }
            else //We have a solution
            {
                //Set the new transverse diameter
                phsPtr_->setTransverseDiameter(maxCost.value());

                //Check whether the problem domain (i.e., StateSpace) or PHS has the smaller measure. Sample the smaller directly and reject out of the larger.
                if (informedSubSpace_->getMeasure() <= phsPtr_->getPhsMeasure())
                {
                    //Variable
                    //The informed subset of the sample as a vector
                    std::vector<double> informedVector(informedSubSpace_->getDimension());

                    //Sample from the state space until the sample is in the PHS
                    do
                    {
                        //Generate a random sample
                        baseSampler_->sampleUniform(statePtr);

                        //Is there an extra "uninformed" subspace to trim off before comparing to the PHS?
                        if ( space_->isCompound() == false )
                        {
                            //No, space_ == informedSubSpace_
                            informedSubSpace_->copyToReals(informedVector, statePtr);
                        }
                        else
                        {
                            //Yes, we need to do some work to extract the subspace
                            informedSubSpace_->copyToReals(informedVector, statePtr->as<CompoundState>()->components[INFORMED_IDX]);
                        }
                    }
                    //Check if the informed state is in the PHS
                    while ( phsPtr_->isInPhs(informedSubSpace_->getDimension(), &informedVector[0]) == false );
                }
                else
                {
                    //Sample from within the PHS until the sample is in the state space
                    do
                    {
                        this->sampleUniformIgnoreBounds(statePtr, maxCost);
                    }
                    while ( space_->satisfiesBounds(statePtr) == false );
                }
            }
        }

        void PathLengthInformedSampler::sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            //Sample from the larger PHS until the sample does not lie within the smaller PHS.
            //Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better
            do
            {
                this->sampleUniform(statePtr, maxCost);
            }
            while ( this->getHeuristicValue(statePtr) < minCost.value() );
        }

        double PathLengthInformedSampler::getHeuristicValue(const State* statePtr)
        {
            //Variable
            //The raw data in the state
            std::vector<double> rawData(informedSubSpace_->getDimension());

            //Get the raw data
            if ( space_->isCompound() == false )
            {
                informedSubSpace_->copyToReals(rawData, statePtr);
            }
            else
            {
                informedSubSpace_->copyToReals(rawData, statePtr->as<CompoundState>()->components[INFORMED_IDX]);
            }

            //Calculate and return the length
            return phsPtr_->getPathLength(informedSubSpace_->getDimension(), &rawData[0]);
        }

        double PathLengthInformedSampler::getInformedMeasure() const
        {
            //Variable
            //The measure of the informed set
            double informedMeasure;

            //It is at least the measure of the PHS:
            informedMeasure = phsPtr_->getPhsMeasure();

            //And if the space is compound, further multiplied by the measure of the uniformed subspace
            if ( space_->isCompound() == true )
            {
                informedMeasure = informedMeasure*uninformedSubSpace_->getMeasure();
            }

            //Return the smaller of the two measures
            return std::min(space_->getMeasure(), informedMeasure);
        }

        double PathLengthInformedSampler::getHypotheticalMeasure(const Cost& hypCost) const
        {
            //Variable
            //The measure of the informed set
            double informedMeasure;

            //It is at least the measure of the PHS:
            informedMeasure = phsPtr_->getPhsMeasure(hypCost.value());

            //And if the space is compound, further multiplied by the measure of the uniformed subspace
            if ( space_->isCompound() == true )
            {
                informedMeasure = informedMeasure*uninformedSubSpace_->getMeasure();
            }

            //Return the smaller of the two measures
            return informedMeasure;
        }

        void PathLengthInformedSampler::setLocalSeed(boost::uint32_t localSeed)
        {
            //Set the seed of my base class, i.e., my rng_ memeber variable
            StateSampler::setLocalSeed(localSeed);

            //Set the seed for my member sub-samplers as well:
            baseSampler_->setLocalSeed( this->getLocalSeed() );

            if ( space_->isCompound() == true )
            {
                uninformedSubSampler_->setLocalSeed( this->getLocalSeed() );
            }
        }

        void PathLengthInformedSampler::sampleUniformIgnoreBounds(State* statePtr, const Cost& maxCost)
        {
            //Variable
            //The informed subset of the sample as a vector
            std::vector<double> informedVector(informedSubSpace_->getDimension());

            //Set the new transverse diameter
            phsPtr_->setTransverseDiameter(maxCost.value());

            //Sample the ellipse
            rng_.uniformProlateHyperspheroid(phsPtr_, informedSubSpace_->getDimension(), &informedVector[0]);

            //If there is an extra "uninformed" subspace, we need to add that to the state before converting the raw vector representation into a state....
            if ( space_->isCompound() == false )
            {
                //No, space_ == informedSubSpace_
                //Copy into the state pointer
                informedSubSpace_->copyFromReals(statePtr, informedVector);
            }
            else
            {
                //Yes, we need to also sample the uninformed subspace
                //Variables
                //A state for the uninformed subspace
                State* uninformedState = uninformedSubSpace_->allocState();

                //Copy the informed subspace into the state pointer
                informedSubSpace_->copyFromReals(statePtr->as<CompoundState>()->components[INFORMED_IDX], informedVector);

                //Sample the uniformed subspace
                uninformedSubSampler_->sampleUniform(uninformedState);

                //Copy the informed subspace into the state pointer
                uninformedSubSpace_->copyState(statePtr->as<CompoundState>()->components[UNINFORMED_IDX], uninformedState);

                //Free the state
                uninformedSubSpace_->freeState(uninformedState);
            }
        }

        void PathLengthInformedSampler::sampleUniformIgnoreBounds(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            //Sample from the larger PHS until the sample does not lie within the smaller PHS.
            //Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better
            do
            {
                this->sampleUniformIgnoreBounds(statePtr, maxCost);
            }
            while ( this->getHeuristicValue(statePtr) < minCost.value() );
        }

    }; //base
};  //ompl
