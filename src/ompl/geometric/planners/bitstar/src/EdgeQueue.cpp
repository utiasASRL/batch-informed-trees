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

#include "ompl/geometric/planners/bitstar/EdgeQueue.h"

//For std::make_pair
#include <utility>

//For exceptions:
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        EdgeQueue::EdgeQueue(bool useParentVertexLookupTables, bool useChildVertexLookupTables, const value_func_t& valueFunc, const sorting_func_t& sortFunc, const threshhold_func_t& threshFunc)
            :   valueFunc_(valueFunc),
                sortFunc_(sortFunc),
                threshFunc_(threshFunc),
                outgoingLookupTables_(useParentVertexLookupTables),
                incomingLookupTables_(useChildVertexLookupTables),
                edgeQueue_( sortFunc_ ), //This tells the edgeQueue_ to use the sortFunc_ for sorting
                outgoingEdges_(),
                incomingEdges_(),
                resortVertices_()
        {
        }

        void EdgeQueue::insert(const vertex_pair_t& newEdge)
        {
            //Call the private helper
            this->insert_helper(newEdge);
        }

        void EdgeQueue::insert(const VertexPtr& pVertex, const VertexPtr& cVertex)
        {
            //Call the private helper
            this->insert_helper( std::make_pair(pVertex, cVertex) );
        }

        void EdgeQueue::remove_to(const VertexPtr& cVertex)
        {
            if (this->empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges to the child:
                    vertex_iter_umap_t::iterator toDeleteIter;

                    //Get the vector of iterators
                    toDeleteIter = incomingEdges_.find(cVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (toDeleteIter != incomingEdges_.end())
                    {
                        //Iterate over the vector removing them from queue
                        for (mmap_iter_list_t::iterator listIter = toDeleteIter->second.begin(); listIter != toDeleteIter->second.end(); ++listIter)
                        {
                            //If the *other* lookup is in use, remove this edge from it. No need to remove from this lookup, as that's being cleared:
                            this->rmOutgoingLookup(*listIter);

                            //Erase from edge queue
                            edgeQueue_.erase(*listIter);
                        }

                        //Clear the list:
                        toDeleteIter->second = mmap_iter_list_t();
                    }
                    //No else, why was this called?
                }
                else
                {
                    throw ompl::Exception("Child lookup is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to remove_to
        }

        void EdgeQueue::remove_from(const VertexPtr& pVertex)
        {
            if (this->empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges from the parent:
                    vertex_iter_umap_t::iterator toDeleteIter;

                    //Get the vector of iterators
                    toDeleteIter = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (toDeleteIter != outgoingEdges_.end())
                    {
                        //Iterate over the vector removing them from queue
                        for (mmap_iter_list_t::iterator listIter = toDeleteIter->second.begin(); listIter != toDeleteIter->second.end(); ++listIter)
                        {
                            //If the *other* lookup is in use, remove this edge from it. No need to remove from this lookup, as that's being cleared:
                            this->rmIncomingLookup(*listIter);

                            //Erase from edge queue
                            edgeQueue_.erase(*listIter);
                        }

                        //Clear the list:
                        toDeleteIter->second = mmap_iter_list_t();
                    }
                    //No else, why was this called?
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to remove_from
        }

        void EdgeQueue::prune_to(const VertexPtr& cVertex, const ompl::base::Cost& costThreshold)
        {
            if (this->empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the key,value of the child-lookup map, i.e., an iterator to a pair whose second is a list of edges to the child (which are actually iterators to the queue):
                    vertex_iter_umap_t::iterator itersToVertex;

                    //Get my incoming edges as a vector of iterators
                    itersToVertex = incomingEdges_.find(cVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (itersToVertex != incomingEdges_.end())
                    {
                        //Variable
                        //The vector of edges to delete in the list:
                        std::vector<mmap_iter_list_t::iterator> listItersToDelete;

                        //Iterate over the incoming edges and record those that are to be deleted
                        for (mmap_iter_list_t::iterator listIter = itersToVertex->second.begin(); listIter != itersToVertex->second.end(); ++listIter)
                        {
                            //Check if it is to be pruned
                            if (threshFunc_((*listIter)->second, costThreshold) == false)
                            {
                                listItersToDelete.push_back(listIter);
                            }
                            //No else, we're not deleting this iterator
                        }

                        //Now, iterate over the list of iterators to delete
                        for (unsigned int i = 0u; i < listItersToDelete.size(); ++i)
                        {
                            //Find and remove the edge iterator from the other lookup table
                            this->rmOutgoingLookup( *listItersToDelete.at(i) );

                            //Erase from the edge queue itself via edge iterator
                            edgeQueue_.erase( *listItersToDelete.at(i) );

                            //And finally erase the lookup iterator from the from lookup. If this was done first, the iterator would be invalidated for the above.
                            itersToVertex->second.erase( listItersToDelete.at(i) );
                        }
                    }
                    //No else, nothing to delete
                }
                else
                {
                    throw ompl::Exception("Child lookup is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to prune_to
        }

        void EdgeQueue::prune_from(const VertexPtr& pVertex, const ompl::base::Cost& costThreshold)
        {
            if (this->empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the key, value of the parent-lookup map, i.e., an iterator to a pair whose second is a list of edges from the child (which are actually iterators to the queue):
                    vertex_iter_umap_t::iterator itersFromVertex;

                    //Get my outgoing edges as a vector of iterators
                    itersFromVertex = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (itersFromVertex != outgoingEdges_.end())
                    {
                        //Variable
                        //The vector of edges to delete in the list:
                        std::vector<mmap_iter_list_t::iterator> listItersToDelete;

                        //Iterate over the incoming edges and record those that are to be deleted
                        for (mmap_iter_list_t::iterator listIter = itersFromVertex->second.begin(); listIter != itersFromVertex->second.end(); ++listIter)
                        {
                            //Check if it is to be pruned
                            if (threshFunc_((*listIter)->second, costThreshold) == false)
                            {
                                listItersToDelete.push_back(listIter);
                            }
                            //No else, we're not deleting this iterator
                        }

                        //Now, iterate over the list of iterators to delete
                        for (unsigned int i = 0u; i < listItersToDelete.size(); ++i)
                        {
                            //Find and remove the edge iterator from the other lookup table
                            this->rmIncomingLookup( *listItersToDelete.at(i) );

                            //Erase from the edge queue itself via edge iterator
                            edgeQueue_.erase( *listItersToDelete.at(i) );

                            //And finally erase the lookup iterator from the from lookup. If this was done first, the iterator would be invalidated for the above.
                            itersFromVertex->second.erase( listItersToDelete.at(i) );
                        }
                    }
                    //No else, nothing to delete
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to prune_from
        }

        EdgeQueue::vertex_pair_t EdgeQueue::front() const
        {
            if (edgeQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty EdgeQueue.");
            }
            //Return the front
            return edgeQueue_.begin()->second;
        }


        EdgeQueue::cost_pair_t EdgeQueue::front_value() const
        {
            if (edgeQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty EdgeQueue.");
            }
            //Return the front
            return edgeQueue_.begin()->first;
        }

        void EdgeQueue::pop_front(vertex_pair_t& bestEdge)
        {
            if (edgeQueue_.empty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty EdgeQueue.");
            }

            //Return the front:
            bestEdge = edgeQueue_.begin()->second;

            //Erase the lookup table (if in use):
            this->eraseLookups(edgeQueue_.begin());

            //Finally erase from the queue:
            edgeQueue_.erase(edgeQueue_.begin());
        }

        EdgeQueue::vertex_pair_t EdgeQueue::pop_front()
        {
            vertex_pair_t rval;

            this->pop_front(rval);

            return rval;
        }

        void EdgeQueue::list(std::vector<vertex_pair_t>& fullQueue)
        {
            //Clear the vector
            fullQueue.clear();

            //I don't think there's a std::copy way to do this, so just iterate
            for( mmap_iter_t eIter = edgeQueue_.begin(); eIter != edgeQueue_.end(); ++eIter )
            {
                fullQueue.push_back(eIter->second);
            }
        }

        void EdgeQueue::resort(const ompl::base::Cost& costThreshold)
        {
            if (outgoingLookupTables_ == true)
            {
                //Iterate over the vector of vertices to resort, reinserting all edges from the flagged vertex, and marking all descendents of the vertex for resorting as well.
                while (resortVertices_.empty() == false)
                {
                    //Variables:
                    //The list of children:
                    std::vector<VertexPtr> resortChildren;
                    //The list of iterators from the vertex:
                    vertex_iter_umap_t::iterator itersFromVertex;

                    //Get the children:
                    resortVertices_.front()->getChildren(resortChildren);

                    //Append to the resort list:
                    std::copy( resortChildren.begin(), resortChildren.end(), std::back_inserter( resortVertices_ ) );

                    //Get my list of outgoing edges
                    itersFromVertex = outgoingEdges_.find( resortVertices_.front() );

                    //Pop off
                    resortVertices_.pop_front();

                    if (itersFromVertex != outgoingEdges_.end())
                    {
                        //Variables
                        //The iterators to the edge queue from this vertex
                        mmap_iter_list_t itersToResort;

                        //Copy the iters to resort
                        itersToResort = itersFromVertex->second;

                        //Clear the outgoing lookup
                        itersFromVertex->second =  mmap_iter_list_t();

                        //Iterate over the list of iters to resort, inserting each one as a new edge, and then removing it as an iterator from the edge queue and the incoming lookup
                        for (mmap_iter_list_t::iterator resortIter = itersToResort.begin(); resortIter != itersToResort.end(); ++resortIter)
                        {
                            //If the *other* lookup is in use, first remove this edge from it. No need to remove from this lookup, as that's beer cleared:
                            this->rmIncomingLookup(*resortIter);

                            //Check if the edge should be reinserted
                            if ( threshFunc_( (*resortIter)->second, costThreshold) == true )
                            {
                                //Call helper to reinsert. Looks after lookups, hint at the location it's coming out of
                                this->insert_helper( (*resortIter)->second, &(*resortIter) );
                            }
                            //No else, prune.

                            //Erase the old one from the edge queue
                            edgeQueue_.erase(*resortIter);
                        }
                    }
                    //No else, no edges from this vertex to requeue
                }
                //Done sorting or empty to star
            }
            else
            {
                throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
            }
        }

        void EdgeQueue::clear()
        {
            //Clear the three containers
            edgeQueue_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();
            resortVertices_.clear();
        }

        bool EdgeQueue::empty() const
        {
            return edgeQueue_.empty();
        }

        unsigned int EdgeQueue::size() const
        {
            return edgeQueue_.size();
        }


        unsigned int EdgeQueue::num_to(const VertexPtr& cVertex) const
        {
            if (this->empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges to the child:
                    vertex_iter_umap_t::const_iterator toIter;

                    //Get the vector of iterators
                    toIter = incomingEdges_.find(cVertex);

                    //Make sure it was found before we dereferencing it:
                    if (toIter != incomingEdges_.end())
                    {
                        return toIter->second.size();
                    }
                    else
                    {
                        return 0u;
                    }
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            else
            {
                return 0u;
            }
        }

        unsigned int EdgeQueue::num_from(const VertexPtr& pVertex) const
        {
            if (this->empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges from the parent:
                    vertex_iter_umap_t::const_iterator toIter;

                    //Get the vector of iterators
                    toIter = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we dereferencing it:
                    if (toIter != outgoingEdges_.end())
                    {
                        return toIter->second.size();
                    }
                    else
                    {
                        return 0u;
                    }
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            else
            {
                return 0u;
            }
        }

        bool EdgeQueue::isSorted() const
        {
            //If there are no vertices to resort, it is sorted
            return resortVertices_.empty();
        }

        void EdgeQueue::markUnsorted(const VertexPtr& vertex)
        {
            resortVertices_.push_back(vertex);
        }

        void EdgeQueue::insert_helper(const vertex_pair_t& newEdge, mmap_iter_t* positionHint /* = NULL*/)
        {
            //Variable:
            //The iterator to the new edge in the queue:
            mmap_iter_t edgeIter;

            //Insert into the edge queue, getting the iter
            if (!positionHint)
            {
                edgeIter = edgeQueue_.insert(std::make_pair(valueFunc_(newEdge), newEdge));
            }
            else
            {
                edgeIter = edgeQueue_.insert(*positionHint, std::make_pair(valueFunc_(newEdge), newEdge));
            }

            if (outgoingLookupTables_ == true)
            {
                //Variable:
                //The iterator to the parent in the lookup map:
                vertex_iter_umap_t::iterator pIter;

                //Find the list of edges from the parent:
                pIter = outgoingEdges_.find(newEdge.first);

                //Was it not found?
                if (pIter == outgoingEdges_.end())
                {
                    //Make an empty vector under this parent:
                    pIter = outgoingEdges_.insert( std::make_pair(newEdge.first, mmap_iter_list_t()) ).first;
                }

                //Push the newly created edge back on the vector:
                pIter->second.push_back(edgeIter);
            }

            if (incomingLookupTables_ == true)
            {
                //Variable:
                //The iterator to the child in the lookup map:
                vertex_iter_umap_t::iterator cIter;

                //Find the list of edges from the child:
                cIter = incomingEdges_.find(newEdge.second);

                //Was it not found?
                if (cIter == incomingEdges_.end())
                {
                    //Make an empty vector under this parent:
                    cIter = incomingEdges_.insert( std::make_pair(newEdge.second, mmap_iter_list_t()) ).first;
                }

                //Push the newly created edge back on the vector:
                cIter->second.push_back(edgeIter);
            }
        }

        void EdgeQueue::eraseLookups(const mmap_iter_t& iterToRmFrmLookup)
        {
            //Erase the entry in the outgoing lookup table:
            this->rmIncomingLookup(iterToRmFrmLookup);

            //and the entry in the ingoing lookup table:
            this->rmOutgoingLookup(iterToRmFrmLookup);
        }

        void EdgeQueue::rmIncomingLookup(const mmap_iter_t& mmapIterToRm)
        {
            if (incomingLookupTables_ == true)
            {
                this->rmLookup(incomingEdges_, mmapIterToRm->second.second, mmapIterToRm);
            }
            //No else
        }

        void EdgeQueue::rmOutgoingLookup(const mmap_iter_t& mmapIterToRm)
        {
            if (outgoingLookupTables_ == true)
            {
                this->rmLookup(outgoingEdges_, mmapIterToRm->second.first, mmapIterToRm);
            }
            //No else
        }

        void EdgeQueue::rmLookup(vertex_iter_umap_t& lookup, const VertexPtr& idx, const mmap_iter_t& mmapIterToRm)
        {
            //Variable:
            //An iterator to the vertex,list pair in the lookup
            vertex_iter_umap_t::iterator iterToVertexListPair;

            //Get the list in the lookup for the given index:
            iterToVertexListPair = lookup.find(idx);

            //Make sure it was actually found before derefencing it:
            if (iterToVertexListPair != lookup.end())
            {
                //Variable:
                //Whether I've found the mmapIterToRm in my list:
                bool found;
                //The iterator to the mmapIterToRm in my list:
                mmap_iter_list_t::iterator iterToList;

                //Start at the front:
                iterToList = iterToVertexListPair->second.begin();

                //Iterate through the list and find mmapIterToRm
                found = false;
                while(found == false && iterToList != iterToVertexListPair->second.end())
                {
                    //Compare the value in the list to the target:
                    if (*iterToList == mmapIterToRm)
                    {
                        //Mark as found:
                        found = true;
                    }
                    else
                    {
                        //Increment the iterator:
                        ++iterToList;
                    }
                }

                if (found == true)
                {
                    iterToVertexListPair->second.erase(iterToList);
                }
                else
                {
                    throw ompl::Exception("Edge iterator not found under given index in lookup hash.");
                }
            }
            else
            {
                throw ompl::Exception("Indexing vertex not found in lookup hash.");
            }
        }
    }
}
