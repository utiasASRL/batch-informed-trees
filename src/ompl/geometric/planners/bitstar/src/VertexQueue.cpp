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

#include "ompl/geometric/planners/bitstar/VertexQueue.h"

//For std::make_pair
#include <utility>

//For exceptions:
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        VertexQueue::VertexQueue(const value_func_t& valueFunc, const sorting_func_t& sortFunc, const threshhold_func_t& threshFunc)
            :   valueFunc_(valueFunc),
                sortFunc_(sortFunc),
                threshFunc_(threshFunc),
                vertexQueue_( sortFunc_ ), //This tells the vertexQueue_ to use the sortFunc_ for sorting
                vertexIterLookup_(),
                resortVertices_()
        {
        }

        void VertexQueue::insert(const VertexPtr& newVertex)
        {
            //Call my helper function
            this->insert_helper(newVertex);
        }

        VertexPtr VertexQueue::front() const
        {
            if (vertexQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty VertexQueue.");
            }
            //Return the front
            return vertexQueue_.begin()->second;
        }

        ompl::base::Cost VertexQueue::front_value() const
        {
            if (vertexQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty VertexQueue.");
            }
            //Return the front
            return vertexQueue_.begin()->first;
        }

        void VertexQueue::pop_front(VertexPtr& bestVertex)
        {
            if (vertexQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty VertexQueue.");
            }

            //Return the front:
            bestVertex = vertexQueue_.begin()->second;

            //Erase from the lookup
            vertexIterLookup_.erase( vertexQueue_.begin()->second );

            //Erase from the queue:
            vertexQueue_.erase(vertexQueue_.begin());
        }

        VertexPtr VertexQueue::pop_front()
        {
            VertexPtr rval;

            this->pop_front(rval);

            return rval;
        }

        void VertexQueue::list(std::vector<VertexPtr>& fullQueue)
        {
            //Clear the vector
            fullQueue.clear();

            //I don't think there's a std::copy way to do this, so just iterate
            for( mmap_iter_t vIter = vertexQueue_.begin(); vIter != vertexQueue_.end(); ++vIter )
            {
                fullQueue.push_back(vIter->second);
            }
        }

        void VertexQueue::resort(const ompl::base::Cost& costThreshold)
        {
            //Iterate over the vector of vertices to resort, resorting the vertex and marking all descendents of the vertex for resorting as well.
            while (resortVertices_.empty() == false)
            {
                //Variables:
                //The list of children:
                std::vector<VertexPtr> resortChildren;

                //Get the children:
                resortVertices_.front()->getChildren(resortChildren);

                //Append to the resort list:
                std::copy( resortChildren.begin(), resortChildren.end(), std::back_inserter( resortVertices_ ) );

                //Erase me:
                this->erase(resortVertices_.front());

                //Check if the edge should be reinserted
                if ( threshFunc_( resortVertices_.front(), costThreshold) == true )
                {
                    //Insert me:
                    this->insert_helper( resortVertices_.front() );
                }

                //Pop me off:
                resortVertices_.pop_front();
            }
            //Done sorting or empty to start
        }

        void VertexQueue::erase(const VertexPtr& oldVertex)
        {
            if (this->empty() == false)
            {
                //Variable
                //The entry in the ordered map
                vertex_iter_umap_t::iterator umapIter;

                //Lookup the multimap iterator for this vertex
                umapIter = vertexIterLookup_.find(oldVertex);

                //Check if it was found
                if (umapIter != vertexIterLookup_.end())
                {
                    //Remove from the multimap
                    vertexQueue_.erase( umapIter->second );

                    //Remove from lookup map
                    vertexIterLookup_.erase(umapIter);
                }
                //No else, nothing to erase
            }
            //No else, nothing to erase
        }

        void VertexQueue::clear()
        {
            //Clear the containers
            vertexQueue_.clear();
            vertexIterLookup_.clear();
            resortVertices_.clear();
        }

        bool VertexQueue::empty() const
        {
            return vertexQueue_.empty();
        }

        unsigned int VertexQueue::size() const
        {
            return vertexQueue_.size();
        }

        bool VertexQueue::isSorted() const
        {
            return resortVertices_.empty();
        }

        void VertexQueue::markUnsorted(const VertexPtr& vertex)
        {
            resortVertices_.push_back(vertex);
        }

        void VertexQueue::insert_helper(const VertexPtr& newVertex, mmap_iter_t* positionHint /*= NULL*/)
        {
            //Variable:
            //The iterator to the new edge in the queue:
            mmap_iter_t vertexIter;

            //Insert into the order map, getting the interator
            if (!positionHint)
            {
                vertexIter = vertexQueue_.insert( std::make_pair(valueFunc_(newVertex), newVertex) );
            }
            else
            {
                vertexIter = vertexQueue_.insert(*positionHint, std::make_pair(valueFunc_(newVertex), newVertex) );
            }

            //Store the iterator in the lookup
            vertexIterLookup_.insert( std::make_pair(newVertex, vertexIter) );
        }
    }
}

