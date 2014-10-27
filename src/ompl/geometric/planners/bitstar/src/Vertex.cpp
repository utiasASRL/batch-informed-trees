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

#include "ompl/geometric/planners/bitstar/Vertex.h"



namespace ompl
{
    namespace geometric
    {
        Vertex::Vertex(const ompl::base::SpaceInformationPtr& si, const ompl::base::OptimizationObjectivePtr& opt, bool root /*= false*/)
          : si_(si),
            opt_(opt),
            state_( si_->allocState() ),
            isRoot_(root),
            isNew_(true),
            parentSPtr_( VertexPtr() ),
            edgeCost_( opt_->infiniteCost() ),
            childWPtrs_(),
            failedWPtrs_()
        {
            if (this->isRoot() == true)
            {
                cost_ = opt_->identityCost();
            }
            else
            {
                cost_ = opt_->infiniteCost();
            }
        }

        Vertex::~Vertex()
        {
            //Free the state on destruction
            si_->freeState(state_);
        }

        ompl::base::State* Vertex::state()
        {
            return state_;
        }

        bool Vertex::isRoot() const
        {
            return isRoot_;
        }

        bool Vertex::hasParent() const
        {
            return bool(parentSPtr_);
        }

        VertexPtr Vertex::getParent() const
        {
            return parentSPtr_;
        }

        void Vertex::addParent(const VertexPtr& newParent, const ompl::base::Cost& edgeInCost, bool updateChildCosts /*= true*/)
        {
            if (this->hasParent() == true)
            {
                throw ompl::Exception("The vertex already has a parent.");
            }
            else if (this->isRoot() == true)
            {
                throw ompl::Exception("The root vertex cannot have a parent.");
            }
            //No else.

            //Store the parent
            parentSPtr_ = newParent;

            //Store the edge cost
            edgeCost_ = edgeInCost;

            //Update my cost
            this->updateCost(updateChildCosts);
        }

        void Vertex::removeParent(bool updateChildCosts /*= true*/)
        {
            if (this->isRoot() == true)
            {
                throw ompl::Exception("The root vertex cannot have a parent.");
            }

            //Clear my parent
            parentSPtr_.reset();

            //Update costs:
            this->updateCost(updateChildCosts);
        }


        bool Vertex::hasChildren() const
        {
            return !childWPtrs_.empty();
        }

        void Vertex::getChildren(std::vector<VertexPtr>& children) const
        {
            for (std::vector<vertex_weak_ptr_t>::const_iterator cIter = childWPtrs_.begin(); cIter != childWPtrs_.end(); ++cIter)
            {
                //Check that the weak pointer hasn't expired
                if (cIter->expired() == true)
                {
                    throw ompl::Exception("A (weak) pointer to a child has expired.");
                }
                else
                {
                    children.push_back(cIter->lock());
                }
            }
        }

        void Vertex::addChild(const VertexPtr& newChild, bool updateChildCosts /*= true*/)
        {
            //Push back the shared_ptr into the vector of weak_ptrs, this makes a weak_ptr copy
            childWPtrs_.push_back( static_cast<vertex_weak_ptr_t>(newChild) );

            if (updateChildCosts == true)
            {
                newChild->updateCost(true);
            }
            //No else, leave the costs out of date.
        }

        void Vertex::removeChild(const VertexPtr& oldChild, bool updateChildCosts /*= true*/)
        {
            //Variables
            //Whether the child has been found (and then deleted);
            bool foundChild;

            //Iterate over the list of children pointers until the child is found. Iterators make erase easier
            foundChild = false;
            for (std::vector<vertex_weak_ptr_t>::iterator cIter = childWPtrs_.begin(); cIter != childWPtrs_.end() && foundChild == false; ++cIter)
            {
                //Check that the weak pointer hasn't expired
                if (cIter->expired() == true)
                {
                    throw ompl::Exception("A (weak) pointer to a child has expired.");
                }
                //No else, weak pointer is valid

                //Check if this is the child we're looking for
                if (cIter->lock() == oldChild)
                {
                    //Remove the child from the vector
                    childWPtrs_.erase(cIter);

                    //Mark as found
                    foundChild = true;

                    //Update the child cost if appropriate
                    if (updateChildCosts == true)
                    {
                        oldChild->updateCost(true);
                    }
                    //No else, leave the costs out of date.
                }
                //No else, move on
            }

            //Throw if we did not find the child
            if (foundChild == false)
            {
                throw ompl::Exception("The given child was not found to be a child of the vertex.");
            }
            //No else, we were successful
        }


        ompl::base::Cost Vertex::getCost() const
        {
            return cost_;
        }


        ompl::base::Cost Vertex::getEdgeInCost() const
        {
            if (this->hasParent() == false)
            {
                throw ompl::Exception("The vertex does not have a parent.");
            }

            return edgeCost_;
        }

        bool Vertex::isConnected() const
        {
            //I am connected if I have a parent or children.
            return this->hasParent() || this->hasChildren();
        }

        bool Vertex::isNew() const
        {
            return isNew_;
        }

        void Vertex::markNew()
        {
            isNew_ = true;
        }

        void Vertex::markOld()
        {
            isNew_ = false;
        }

        void Vertex::markAsFailedChild(const VertexPtr& failedChild)
        {
            failedWPtrs_.insert( static_cast<vertex_weak_ptr_t>(failedChild) );
        }


        bool Vertex::hasAlreadyFailed(const VertexPtr& potentialChild) const
        {
            //Return true if there is more than 0 of this pointer.
            return failedWPtrs_.count( static_cast<vertex_weak_ptr_t>(potentialChild) ) > 0u;
        }

        void Vertex::updateCost(bool cascadeUpdates /*= true*/)
        {
            if (this->isRoot() == true)
            {
                //Am I root? -- I don't really know how this would ever be called, but ok.
                cost_ = opt_->identityCost();
            }
            else if (this->isConnected() == false)
            {
                //Am I disconnected?
                cost_ = opt_->infiniteCost();
            }
            else
            {
                //Do I have a parent?
                if (parentSPtr_)
                {
                    //I have a parent, so my cost is my parent cost + my edge cost to the parent
                    cost_ = opt_->combineCosts(parentSPtr_->getCost(), edgeCost_);
                }
                else
                {
                    //I have children (as I am not disconnected) but no parent. Set my cost to infinity, but assert that I'm not updating my children.
//                    cost_ = opt_->infiniteCost();
                    cost_ = opt_->identityCost();

                    if (cascadeUpdates == true)
                    {
                        throw ompl::Exception("A non-root vertex with no parent, but with children, is having it's cost updated and being told to cascade it's updates.");
                    }
                }
            }

            //Am I updating my children?
            if (cascadeUpdates == true)
            {
                //Now, iterate over my list of children and tell each one to update its own damn cost:
                for (unsigned int i = 0u; i < childWPtrs_.size(); ++i)
                {
                    //Check that it hasn't expired
                    if (childWPtrs_.at(i).expired() == true)
                    {
                        throw ompl::Exception("A (weak) pointer to a child has expired.");
                    }
                    //No else, weak pointer is valid

                    //Get a lock and tell the child to update:
                    childWPtrs_.at(i).lock()->updateCost(true);
                }
            }
            //No else, do not update the children. I hope the caller knows what they're doing.
        }
    }//geometric
}//ompl
