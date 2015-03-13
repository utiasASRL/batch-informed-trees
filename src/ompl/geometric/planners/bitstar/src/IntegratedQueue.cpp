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

//Myself:
#include "ompl/geometric/planners/bitstar/IntegratedQueue.h"

//OMPL:
//For exceptions:
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        IntegratedQueue::IntegratedQueue(const ompl::base::OptimizationObjectivePtr& opt, const neighbourhood_func_t& nearSamplesFunc, const neighbourhood_func_t& nearVerticesFunc, const vertex_heuristic_func_t& lowerBoundHeuristicVertex, const vertex_heuristic_func_t& currentHeuristicVertex, const edge_heuristic_func_t& lowerBoundHeuristicEdge, const edge_heuristic_func_t& currentHeuristicEdge, const edge_heuristic_func_t& currentHeuristicEdgeTarget)
            :   opt_(opt),
                nearSamplesFunc_(nearSamplesFunc),
                nearVerticesFunc_(nearVerticesFunc),
                lowerBoundHeuristicVertexFunc_(lowerBoundHeuristicVertex),
                currentHeuristicVertexFunc_(currentHeuristicVertex),
                lowerBoundHeuristicEdgeFunc_(lowerBoundHeuristicEdge),
                currentHeuristicEdgeFunc_(currentHeuristicEdge),
                currentHeuristicEdgeTargetFunc_(currentHeuristicEdgeTarget),
                useFailureTracking_(false),
                outgoingLookupTables_(true),
                incomingLookupTables_(true),
                vertexQueue_( boost::bind(&IntegratedQueue::vertexQueueComparison, this, _1, _2) ), //This tells the vertexQueue_ to use the vertexQueueComparison for sorting
                vertexToExpand_( vertexQueue_.begin() ),
                edgeQueue_( boost::bind(&IntegratedQueue::edgeQueueComparison, this, _1, _2) ), //This tells the edgeQueue_ to use the edgeQueueComparison for sorting
                vertexIterLookup_(),
                outgoingEdges_(),
                incomingEdges_(),
                resortVertices_(),
                costThreshold_( std::numeric_limits<double>::infinity() ) //Purposeful gibberish
        {
            //The cost threshold:
            costThreshold_ = opt_->infiniteCost();
        }



        void IntegratedQueue::insertVertex(const VertexPtr& newVertex)
        {
            //Insert the vertex:
            this->vertexInsertHelper(newVertex, true);
        }



        void IntegratedQueue::insertEdge(const vertex_pair_t& newEdge)
        {
            //Call my helper function:
            this->edgeInsertHelper(newEdge, edgeQueue_.end());
        }



        void IntegratedQueue::eraseVertex(const VertexPtr& oldVertex, bool removeIncomingEdges, bool removeOutgoingEdges)
        {
            this->vertexRemoveHelper(oldVertex, removeIncomingEdges, removeOutgoingEdges, resortVertices_.end());
        }



        VertexPtr IntegratedQueue::frontVertex()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            //Update the queue:
            this->updateQueue();

            //Return the front edge
            return vertexQueue_.begin()->second;
        }



        IntegratedQueue::vertex_pair_t IntegratedQueue::frontEdge()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            //Update the queue:
            this->updateQueue();

            //Return the front edge
            return edgeQueue_.begin()->second;
        }



        ompl::base::Cost IntegratedQueue::frontVertexValue()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            //Update the queue:
            this->updateQueue();

            //Return the front value
            return vertexQueue_.begin()->first;
        }



        IntegratedQueue::cost_pair_t IntegratedQueue::frontEdgeValue()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            //Update the queue:
            this->updateQueue();

            //Return the front value
            return edgeQueue_.begin()->first;
        }



        void IntegratedQueue::popFrontEdge(vertex_pair_t& bestEdge)
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty IntegratedQueue.");
            }

            //Update the queue:
            this->updateQueue();

            //Return the front:
            bestEdge = edgeQueue_.begin()->second;

            //Erase the edge:
            this->edgeRemoveHelper(edgeQueue_.begin(), true, true);
        }



        IntegratedQueue::vertex_pair_t IntegratedQueue::popFrontEdge()
        {
            vertex_pair_t rval;

            this->popFrontEdge(rval);

            return rval;
        }



        void IntegratedQueue::setThreshold(const ompl::base::Cost& costThreshold)
        {
            costThreshold_ = costThreshold;
        }



        void IntegratedQueue::removeEdgesTo(const VertexPtr& cVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges to the child:
                    vertex_edge_queue_iter_umap_t::iterator toDeleteIter;

                    //Get the vector of iterators
                    toDeleteIter = incomingEdges_.find(cVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (toDeleteIter != incomingEdges_.end())
                    {
                        //Iterate over the vector removing them from queue
                        for (edge_queue_iter_list_t::iterator listIter = toDeleteIter->second.begin(); listIter != toDeleteIter->second.end(); ++listIter)
                        {
                            //Erase the edge, removing it from the *other* lookup. No need to remove from this lookup, as that's being cleared:
                            this->edgeRemoveHelper(*listIter, false, true);
                        }

                        //Clear the list:
                        toDeleteIter->second = edge_queue_iter_list_t();
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



        void IntegratedQueue::removeEdgesFrom(const VertexPtr& pVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges from the parent:
                    vertex_edge_queue_iter_umap_t::iterator toDeleteIter;

                    //Get the vector of iterators
                    toDeleteIter = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (toDeleteIter != outgoingEdges_.end())
                    {
                        //Iterate over the vector removing them from queue
                        for (edge_queue_iter_list_t::iterator listIter = toDeleteIter->second.begin(); listIter != toDeleteIter->second.end(); ++listIter)
                        {
                            //Erase the edge, removing it from the *other* lookup. No need to remove from this lookup, as that's being cleared:
                            this->edgeRemoveHelper(*listIter, true, false);
                        }

                        //Clear the list:
                        toDeleteIter->second = edge_queue_iter_list_t();
                    }
                    //No else, why was this called?
                }
                else
                {
                    throw ompl::Exception("Removing edges in the queue coming from a vertex requires parent vertex lookup, which is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to remove_from
        }



        void IntegratedQueue::pruneEdgesTo(const VertexPtr& cVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the key,value of the child-lookup map, i.e., an iterator to a pair whose second is a list of edges to the child (which are actually iterators to the queue):
                    vertex_edge_queue_iter_umap_t::iterator itersToVertex;

                    //Get my incoming edges as a vector of iterators
                    itersToVertex = incomingEdges_.find(cVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (itersToVertex != incomingEdges_.end())
                    {
                        //Variable
                        //The vector of edges to delete in the list:
                        std::vector<edge_queue_iter_list_t::iterator> listItersToDelete;

                        //Iterate over the incoming edges and record those that are to be deleted
                        for (edge_queue_iter_list_t::iterator listIter = itersToVertex->second.begin(); listIter != itersToVertex->second.end(); ++listIter)
                        {
                            //Check if it is to be pruned
                            if ( this->edgePruneCondition((*listIter)->second) == true )
                            {
                                listItersToDelete.push_back(listIter);
                            }
                            //No else, we're not deleting this iterator
                        }

                        //Now, iterate over the list of iterators to delete
                        for (unsigned int i = 0u; i < listItersToDelete.size(); ++i)
                        {
                            //Remove the edge and the edge iterator from the other lookup table:
                            this->edgeRemoveHelper( *listItersToDelete.at(i), false, true);

                            //And finally erase the lookup iterator from the from lookup. If this was done first, the iterator would be invalidated for the above.
                            itersToVertex->second.erase( listItersToDelete.at(i) );
                        }
                    }
                    //No else, nothing to delete
                }
                else
                {
                    throw ompl::Exception("Removing edges in the queue going to a vertex requires child vertex lookup, which is not enabled for this instance of the container.");
                }
            }
            //No else, nothing to prune_to
        }



        void IntegratedQueue::pruneEdgesFrom(const VertexPtr& pVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the key, value of the parent-lookup map, i.e., an iterator to a pair whose second is a list of edges from the child (which are actually iterators to the queue):
                    vertex_edge_queue_iter_umap_t::iterator itersFromVertex;

                    //Get my outgoing edges as a vector of iterators
                    itersFromVertex = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we start dereferencing it:
                    if (itersFromVertex != outgoingEdges_.end())
                    {
                        //Variable
                        //The vector of edges to delete in the list:
                        std::vector<edge_queue_iter_list_t::iterator> listItersToDelete;

                        //Iterate over the incoming edges and record those that are to be deleted
                        for (edge_queue_iter_list_t::iterator listIter = itersFromVertex->second.begin(); listIter != itersFromVertex->second.end(); ++listIter)
                        {
                            //Check if it is to be pruned
                            if ( this->edgePruneCondition((*listIter)->second) == true )
                            {
                                listItersToDelete.push_back(listIter);
                            }
                            //No else, we're not deleting this iterator
                        }

                        //Now, iterate over the list of iterators to delete
                        for (unsigned int i = 0u; i < listItersToDelete.size(); ++i)
                        {
                            //Remove the edge and the edge iterator from the other lookup table:
                            this->edgeRemoveHelper( *listItersToDelete.at(i), true, false );

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



        void IntegratedQueue::markVertexUnsorted(const VertexPtr& vertex)
        {
            resortVertices_.push_back(vertex);
        }



        void IntegratedQueue::resort()
        {
            if (outgoingLookupTables_ == true)
            {
                //Iterate over the vector of vertices to resort, reinserting each flagged vertex, all its outgoing edges, and marking all its descendents for resorting as well.
                while (resortVertices_.empty() == false)
                {
                    //Variables:
                    //The vertex to reinsert:
                    VertexPtr unorderedVertex;
                    //The list of children:
                    std::vector<VertexPtr> resortChildren;

                    //Get the vertex:
                    unorderedVertex = resortVertices_.front();

                    //Get the children:
                    unorderedVertex->getChildren(resortChildren);

                    //Append the children to the resort list:
                    std::copy( resortChildren.begin(), resortChildren.end(), std::back_inserter( resortVertices_ ) );

                    //Check if the vertex will be reinserted
                    if ( this->vertexPruneCondition(unorderedVertex) == false )
                    {
                        //It will be reinserted

                        //Variables:
                        //Whether the vertex is currently expanded
                        bool expanded;
                        //The list of iterators from the vertex:
                        vertex_edge_queue_iter_umap_t::iterator itersFromVertex;

                        //Test if I have been expanded:
                        if (vertexToExpand_ == vertexQueue_.end())
                        {
                            //The token is at the end, therefore this vertex is in front of it:
                            expanded = true;
                        }
                        else if ( this->vertexQueueComparison(vertexIterLookup_.find(unorderedVertex)->second->first, vertexToExpand_->first) == true )
                        {
                            //The vertexQueueCondition says that this vertex is in front of the current token:
                            expanded = true;
                        }

                        //Remove myself from the queue and my entry in the resorting list, but do not touch my edges:
                        this->vertexRemoveHelper(unorderedVertex, false, false, resortVertices_.begin());

                        //Reinsert me. If I have not yet been expanded, then I should be when I cross the token:
                        this->vertexInsertHelper(unorderedVertex, expanded == false);

                        //Get my list of outgoing edges
                        itersFromVertex = outgoingEdges_.find(unorderedVertex);

                        //Reinsert the edges:
                        if (itersFromVertex != outgoingEdges_.end())
                        {
                            //Variables
                            //The iterators to the edge queue from this vertex
                            edge_queue_iter_list_t itersToResort;

                            //Copy the iters to resort
                            itersToResort = itersFromVertex->second;

                            //Clear the outgoing lookup
                            itersFromVertex->second =  edge_queue_iter_list_t();

                            //Iterate over the list of iters to resort, inserting each one as a new edge, and then removing it as an iterator from the edge queue and the incoming lookup
                            for (edge_queue_iter_list_t::iterator resortIter = itersToResort.begin(); resortIter != itersToResort.end(); ++resortIter)
                            {
                                //Check if the edge should be reinserted
                                if ( this->edgePruneCondition((*resortIter)->second) == false )
                                {
                                    //Call helper to reinsert. Looks after lookups, hint at the location it's coming out of
                                    this->edgeInsertHelper( (*resortIter)->second, *resortIter );
                                }
                                //No else, prune.

                                //Remove the old edge and its entry in the incoming lookup. No need to remove from this lookup, as that's been cleared:
                                this->edgeRemoveHelper(*resortIter, true, false);
                            }
                        }
                        //No else, no edges from this vertex to requeue
                    }
                    else
                    {
                        //It will not be reinserted
                        //Remove everything about myself:
                        this->vertexRemoveHelper(unorderedVertex, true, true, resortVertices_.begin());
                    }
                }
                //Done sorting or empty to star
            }
            else
            {
                throw ompl::Exception("Parent lookup is required for edge queue resorting, but is not enabled for this instance of the container.");
            }
        }



        void IntegratedQueue::reset()
        {
            //Make sure the queue is "finished":
            this->finish();

            //Restart the expansion queue:
            vertexToExpand_ = vertexQueue_.begin();
        }



        void IntegratedQueue::finish()
        {
            //Clear the edge containers:
            edgeQueue_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            //Do NOT clear:
            //  -  resortVertices_ (they may still need to be resorted)
            //  - vertexIterLookup_ (it's still valid)
        }



        void IntegratedQueue::clear()
        {
            //Clear:
            //The vertex queue:
            vertexQueue_.clear();
            vertexToExpand_ = vertexQueue_.begin();

            //The edge queue:
            edgeQueue_.clear();

            //The lookups:
            vertexIterLookup_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            //The resort list:
            resortVertices_.clear();

            //The cost threshold:
            costThreshold_ = opt_->infiniteCost();
        }




        bool IntegratedQueue::vertexPruneCondition(const VertexPtr& state) const
        {
            //Threshold should always be g_t(x_g)
            //As the sample is in the graph (and therefore could be part of g_t), prune iff g^(v) + h^(v) > g_t(x_g)
            //g^(v) + h^(v) <= g_t(x_g)
            return this->isCostWorseThan(lowerBoundHeuristicVertexFunc_(state), costThreshold_);
        }



        bool IntegratedQueue::edgePruneCondition(const vertex_pair_t& edge) const
        {
            bool rval;
            //Threshold should always be g_t(x_g)

            // g^(v) + c^(v,x) + h^(x) > g_t(x_g)?
            rval = this->isCostWorseThan(lowerBoundHeuristicEdgeFunc_(edge), costThreshold_);


            //If the child is connected already, we need to check if we could do better than it's current connection. But only if we're not pruning based on the first check
            if (edge.second->hasParent() == true && rval == false)
            {
                //g^(v) + c^(v,x) > g_t(x)
                //rval = this->isCostWorseThan(opt_->combineCosts(this->costToComeHeuristic(edge.first), this->edgeCostHeuristic(edge)), edge.second->getCost()); //Ever rewire?
                //g_t(v) + c^(v,x) > g_t(x)
                rval = this->isCostWorseThan(currentHeuristicEdgeTargetFunc_(edge), edge.second->getCost()); //Currently rewire?
            }

            return  rval;
        }



        unsigned int IntegratedQueue::numEdges() const
        {
            return edgeQueue_.size();
        }



        unsigned int IntegratedQueue::numVertices() const
        {
            //Variables:
            //The number of vertices left to expand:
            unsigned int numToExpand;

            //Start at 0:
            numToExpand = 0u;

            //Iterate until the end:
            for (cost_vertex_multimap_t::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                //Increment counter:
                ++numToExpand;
            }

            //Return
            return numToExpand;
        }



        unsigned int IntegratedQueue::numEdgesTo(const VertexPtr& cVertex) const
        {
            //Variables:
            //The number of edges to:
            unsigned int rval;

            //Start at 0:
            rval = 0u;

            //Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges to the child:
                    vertex_edge_queue_iter_umap_t::const_iterator toIter;

                    //Get the vector of iterators
                    toIter = incomingEdges_.find(cVertex);

                    //Make sure it was found before we dereferencing it:
                    if (toIter != incomingEdges_.end())
                    {
                        rval = toIter->second.size();
                    }
                    //No else, there are none.
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            //No else, there is nothing.

            //Return:
            return rval;
        }



        unsigned int IntegratedQueue::numEdgesFrom(const VertexPtr& pVertex) const
        {
            //Variables:
            //The number of edges to:
            unsigned int rval;

            //Start at 0:
            rval = 0u;

            //Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    //Variable:
                    //The iterator to the vector of edges from the parent:
                    vertex_edge_queue_iter_umap_t::const_iterator toIter;

                    //Get the vector of iterators
                    toIter = outgoingEdges_.find(pVertex);

                    //Make sure it was found before we dereferencing it:
                    if (toIter != outgoingEdges_.end())
                    {
                        rval = toIter->second.size();
                    }
                    //No else, 0u.
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            //No else, there is nothing.

            //Return
            return rval;
        }



        bool IntegratedQueue::isSorted() const
        {
            return resortVertices_.empty();
        }



        bool IntegratedQueue::isEmpty()
        {
            //Expand if the edge queue is empty but the vertex queue is not:
            while (edgeQueue_.empty() && vertexToExpand_ != vertexQueue_.end())
            {
                //Expand the next vertex, this pushes the token:
                this->expandNextVertex();
            }

            //Return whether the edge queue is empty:
            return edgeQueue_.empty();
        }



        void IntegratedQueue::listVertices(std::vector<VertexPtr>* vertexQueue)
        {
            //Clear the given list:
            vertexQueue->clear();

            //Iterate until the end, pushing back:
            for (cost_vertex_multimap_t::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                //Push back:
                vertexQueue->push_back(vIter->second);
            }
        }



        void IntegratedQueue::listEdges(std::vector<vertex_pair_t>* edgeQueue)
        {
            //Clear the vector
            edgeQueue->clear();

            //I don't think there's a std::copy way to do this, so just iterate
            for( cost_pair_vertex_pair_multimap_t::const_iterator eIter = edgeQueue_.begin(); eIter != edgeQueue_.end(); ++eIter )
            {
                edgeQueue->push_back(eIter->second);
            }
        }









        void IntegratedQueue::updateQueue()
        {
            //Variables:
            //Whether to expand:
            bool expand;

            expand = true;
            while ( expand == true )
            {
                //Check that there are vertices to expand
                if (vertexToExpand_ != vertexQueue_.end())
                {
                    //Expand a vertex if the edge queue is empty, or the vertex could place a better edge into it:
                    if (edgeQueue_.empty() == true)
                    {
                        //The edge queue is empty, any edge is better than this!
                        this->expandNextVertex();
                    }
                    else if (this->isCostBetterThanOrEquivalentTo( vertexToExpand_->first, edgeQueue_.begin()->first.first ) == true)
                    {
                        //The vertex *could* give a better edge than our current best edge:
                        this->expandNextVertex();
                    }
                    else
                    {
                        //We are done expanding for now:
                        expand = false;
                    }
                }
                else
                {
                    //There are no vertices left to expand
                    expand = false;
                }
            }
        }


        void IntegratedQueue::expandNextVertex()
        {
            //Should we expand the next vertex? Will it be pruned?
            if (this->vertexPruneCondition(vertexToExpand_->second) == false)
            {
                //Expand the vertex in the front:
                this->expandVertex(vertexToExpand_->second);

//                unsigned int beforeEdge = edgeQueue_.size();
//                std::cout << "Expanding: " << vertexToExpand_->first.value() << " -> ";

                //Increment the vertex token:
                ++vertexToExpand_;

//                if (vertexToExpand_ != vertexQueue_.end())
//                {
//                    std::cout << vertexToExpand_->first.value();
//                }
//                else
//                {
//                    std::cout << "infty";
//                }
//
//                std::cout << " creating " << edgeQueue_.size() - beforeEdge << " new edges." << std::endl;
            }
            else
            {
                //The next vertex would get pruned, so just jump to the end:
                vertexToExpand_ = vertexQueue_.end();
            }

        }


        void IntegratedQueue::expandVertex(const VertexPtr& vertex)
        {
            //Should we expand this vertex?
            if (this->vertexPruneCondition(vertex) == false)
            {
                //Variables:
                //The vector of nearby samples (either within r or the k-nearest)
                std::vector<VertexPtr> neighbourSamples;

                //Get the set of nearby free states:
                nearSamplesFunc_(vertex, &neighbourSamples);

                //Iterate over the vector and add only those who could ever provide a better solution:
                for (unsigned int i = 0u; i < neighbourSamples.size(); ++i)
                {
                    //Attempt to queue the edge
                    this->queueupEdge(vertex, neighbourSamples.at(i));
                }

                //If it is a new vertex, we also add rewiring candidates:
                if (vertex->isNew() == true)
                {
                    //Variables:
                    //The vector of vertices within r of the vertexf
                    std::vector<VertexPtr> neighbourVertices;

                    //Get the set of nearby free states:
                    nearVerticesFunc_(vertex, &neighbourVertices);

                    //Iterate over the vector and add only those who could ever provide a better solution:
                    for (unsigned int i = 0u; i < neighbourVertices.size(); ++i)
                    {
                        //Make sure it is not the root or myself.
                        if (neighbourVertices.at(i)->isRoot() == false && neighbourVertices.at(i) != vertex)
                        {
                            //Make sure I am not already the parent or child
                            if (neighbourVertices.at(i)->getParent() != vertex && neighbourVertices.at(i) != vertex->getParent())
                            {
                                //Attempt to queue the edge:
                                this->queueupEdge(vertex, neighbourVertices.at(i));
                            }
                            //No else
                        }
                        //No else
                    }

                    //Mark the vertex as old
                    vertex->markOld();
                }
                //No else
            }
            //No else
        }



        void IntegratedQueue::queueupEdge(const VertexPtr& parent, const VertexPtr& child)
        {
            //Variables:
            //A bool to store the conditional failed edge check
            bool previouslyFailed;

            //See if we're checking for previous failure:
            if (useFailureTracking_ == true)
            {
                previouslyFailed = parent->hasAlreadyFailed(child);
            }
            else
            {
                previouslyFailed = false;
            }

            //Make sure the edge has not already failed
            if (previouslyFailed == false)
            {
                //Variable:
                //The edge:
                vertex_pair_t newEdge;

                //Make the edge
                newEdge = std::make_pair(parent, child);

                //Should this edge be in the queue? I.e., is it *not* due to be pruned:
                if (this->edgePruneCondition(newEdge) == false)
                {
                    this->edgeInsertHelper(newEdge, edgeQueue_.end());
                }
                //No else, we assume that it's better to calculate this condition multiple times than have the list of failed sets become too large...?
            }
            //No else
        }



        void IntegratedQueue::vertexInsertHelper(const VertexPtr& newVertex, bool expandIfBeforeToken)
        {
            //Variable:
            //The iterator to the new edge in the queue:
            vertex_queue_iter_t vertexIter;

            //Insert into the order map, getting the interator
            vertexIter = vertexQueue_.insert( std::make_pair(this->vertexQueueValue(newVertex), newVertex) );

            //Store the iterator in the lookup
            vertexIterLookup_.insert( std::make_pair(newVertex, vertexIter) );

            //Check if we are in front of the token and expand if so:
            if (vertexQueue_.size() == 1u)
            {
                //If the vertex queue is now of size 1, that means that this was the first vertex. Set the token to it and don't even think of expanding anything:
                vertexToExpand_ = vertexQueue_.begin();
            }
            else if (expandIfBeforeToken == true)
            {
                /*
                There are 3ish cases:
                    1 The new vertex is immediately before the token.
                        a The token is not at the end: Don't expand and shift the token to the new vertex.
                        b The token is at the end: Don't expand and shift the token to the new vertex.
                    2 The new vertex is before the token, but *not* immediately (i.e., there are vertices between it):
                        a The token is at the end: Expand the vertex
                        b The token is not at the end: Expand the vertex
                    3 The new vertex is after the token: Don't expand. It cleanly goes into the list of vertices to expand
                Note: By shifting the token, we assure that if the new vertex is better than the best edge, it will get expanded on the next pop.

                The cases look like this (-: expanded vertex, x: unexpanded vertex, X: token (next to expand), *: new vertex):
                We represent the token at the end with no X in the line:

                    1a: ---*Xxx   ->   ---Xxxx
                    1b: ------*   ->   ------X
                    2a: ---*---   ->   -------
                    2b: --*-Xxx   ->   ----Xxx
                    3: ---Xx*x   ->   ---Xxxx
                */

                //Variable:
                //The vertex before the token. Remember that since we have already added the new vertex, this could be ourselves:
                vertex_queue_iter_t preToken;

                //Get the vertex before the current token:
                preToken = vertexToExpand_;
                --preToken;

                //Check if we are immediately before: (1a & 1b)
                if (preToken == vertexIter)
                {
//                    if (vertexToExpand_ != vertexQueue_.end())
//                    {
//                        std::cout << "1a: " << vertexToExpand_->first.value() << " -> " << vertexIter->first.value() << std::endl;
//                    }
//                    else
//                    {
//                        std::cout << "1b: infty  -> " << vertexIter->first.value() << std::endl;
//                    }
                    //The vertex before the token is the newly added vertex. Therefore we can just move the token up to the newly added vertex:
                    vertexToExpand_ = vertexIter;
                }
                else
                {
                    //We are not immediately before the token.

                    //Check if the token is at the end (2a)
                    if (vertexToExpand_ == vertexQueue_.end())
                    {
                        //It is. We've expanded the whole queue, and the new vertex isn't at the end of the queue. Expand!
                        this->expandVertex(newVertex);
//                        std::cout << "2a: " << vertexIter->first.value() << " < infty" << std::endl;
                    }
                    else
                    {
                        //The token is not at the end. That means we can safely dereference it:
                        //Are we in front of it (2b)?
                        if ( this->vertexQueueComparison(this->vertexQueueValue(newVertex), vertexToExpand_->first) == true )
                        {
                            //We're before it, so expand it:
                            this->expandVertex(newVertex);
//                            std::cout << "2b: " << vertexIter->first.value() << " < " << vertexToExpand_->first.value() << std::endl;
                        }
//                        else
//                        {
//                            std::cout << "3: " << vertexIter->first.value() << " > " << vertexToExpand_->first.value() << std::endl;
//                        }
                        //No else, the vertex is behind the current token (3) and will get expanded as necessary.
                    }
                }
            }
        }



        void IntegratedQueue::vertexRemoveHelper(const VertexPtr& oldVertex, bool removeIncomingEdges, bool removeOutgoingEdges, std::list<VertexPtr>::iterator resortIter)
        {
            //Check if there's anything to delete:
            if (vertexQueue_.empty() == false)
            {
                //Variable
                //The entry in the ordered map
                vertex_vertex_queue_iter_umap_t::iterator umapIter;

                //Lookup the multimap iterator for this vertex
                umapIter = vertexIterLookup_.find(oldVertex);

                //Check if it was found
                if (umapIter != vertexIterLookup_.end())
                {
                    //Check if we need to move the expansion token:
                    if (umapIter->second == vertexToExpand_)
                    {
                        //It is the token, move it to the next:
                        ++vertexToExpand_;
                    }
                    //No else, not the token.

                    //Remove from the multimap
                    vertexQueue_.erase( umapIter->second );

                    //Remove from lookup map
                    vertexIterLookup_.erase(umapIter);

                    //Remove all edges from the state if requested;
                    if (removeOutgoingEdges == true)
                    {
                        this->removeEdgesFrom(oldVertex);
                    }

                    //Also Remove from the incoming edge container if requested:
                    if (removeIncomingEdges == true)
                    {
                        this->removeEdgesTo(oldVertex);
                    }

                    //Check if we've been given a listing in the resort list to remove:
                    if (resortIter != resortVertices_.end())
                    {
                        //Remove it:
                        resortVertices_.erase(resortIter);
                    }
                    else if (resortVertices_.empty() == false)
                    {
                        //Throw a complaint:
                        throw ompl::Exception("Removing a vertex from a non-sorted queue without specifying the resort element to remove.");
                    }
                }
                else
                {
                    throw ompl::Exception("Removing a nonexistent vertex.");
                }
            }
            else
            {
                throw ompl::Exception("Removing a nonexistent vertex.");
            }
        }



        void IntegratedQueue::edgeInsertHelper(const vertex_pair_t& newEdge, edge_queue_iter_t positionHint)
        {
            //Variable:
            //The iterator to the new edge in the queue:
            edge_queue_iter_t edgeIter;

            //Insert into the edge queue, getting the iter
            if (positionHint == edgeQueue_.end())
            {
                //No hint, insert:
                edgeIter = edgeQueue_.insert(std::make_pair(this->edgeQueueValue(newEdge), newEdge));
            }
            else
            {
                //Insert with hint:
                edgeIter = edgeQueue_.insert(positionHint, std::make_pair(this->edgeQueueValue(newEdge), newEdge));
            }

            if (outgoingLookupTables_ == true)
            {
                //Variable:
                //The iterator to the parent in the lookup map:
                vertex_edge_queue_iter_umap_t::iterator pIter;

                //Find the list of edges from the parent:
                pIter = outgoingEdges_.find(newEdge.first);

                //Was it not found?
                if (pIter == outgoingEdges_.end())
                {
                    //Make an empty vector under this parent:
                    pIter = outgoingEdges_.insert( std::make_pair(newEdge.first, edge_queue_iter_list_t()) ).first;
                }

                //Push the newly created edge back on the vector:
                pIter->second.push_back(edgeIter);
            }

            if (incomingLookupTables_ == true)
            {
                //Variable:
                //The iterator to the child in the lookup map:
                vertex_edge_queue_iter_umap_t::iterator cIter;

                //Find the list of edges from the child:
                cIter = incomingEdges_.find(newEdge.second);

                //Was it not found?
                if (cIter == incomingEdges_.end())
                {
                    //Make an empty vector under this parent:
                    cIter = incomingEdges_.insert( std::make_pair(newEdge.second, edge_queue_iter_list_t()) ).first;
                }

                //Push the newly created edge back on the vector:
                cIter->second.push_back(edgeIter);
            }
        }



        void IntegratedQueue::edgeRemoveHelper(const edge_queue_iter_t& oldEdgeIter, bool rmIncomingLookup, bool rmOutgoingLookup)
        {
            //Erase the lookup tables:
            if (rmIncomingLookup == true)
            {
                //Erase the entry in the outgoing lookup table:
                this->rmIncomingLookup(oldEdgeIter);
            }
            //No else

            if (rmOutgoingLookup == true)
            {
                //Erase  the entry in the ingoing lookup table:
                this->rmOutgoingLookup(oldEdgeIter);
            }
            //No else

            //Finally erase from the queue:
            edgeQueue_.erase(oldEdgeIter);
        }



        void IntegratedQueue::rmIncomingLookup(const edge_queue_iter_t& mmapIterToRm)
        {
            if (incomingLookupTables_ == true)
            {
                this->rmLookup(incomingEdges_, mmapIterToRm->second.second, mmapIterToRm);
            }
            //No else
        }



        void IntegratedQueue::rmOutgoingLookup(const edge_queue_iter_t& mmapIterToRm)
        {
            if (outgoingLookupTables_ == true)
            {
                this->rmLookup(outgoingEdges_, mmapIterToRm->second.first, mmapIterToRm);
            }
            //No else
        }



        void IntegratedQueue::rmLookup(vertex_edge_queue_iter_umap_t& lookup, const VertexPtr& idx, const edge_queue_iter_t& mmapIterToRm)
        {
            //Variable:
            //An iterator to the vertex,list pair in the lookup
            vertex_edge_queue_iter_umap_t::iterator iterToVertexListPair;

            //Get the list in the lookup for the given index:
            iterToVertexListPair = lookup.find(idx);

            //Make sure it was actually found before derefencing it:
            if (iterToVertexListPair != lookup.end())
            {
                //Variable:
                //Whether I've found the mmapIterToRm in my list:
                bool found;
                //The iterator to the mmapIterToRm in my list:
                edge_queue_iter_list_t::iterator iterToList;

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



















        ompl::base::Cost IntegratedQueue::vertexQueueValue(const VertexPtr& vertex) const
        {
            return currentHeuristicVertexFunc_(vertex);
        }



        IntegratedQueue::cost_pair_t IntegratedQueue::edgeQueueValue(const vertex_pair_t& edge) const
        {
            return std::make_pair(currentHeuristicEdgeFunc_(edge), edge.first->getCost());
        }



        bool IntegratedQueue::vertexQueueComparison(const ompl::base::Cost& lhs, const ompl::base::Cost& rhs) const
        {
            //lhs < rhs?
            return this->isCostBetterThan(lhs, rhs);
        }



        bool IntegratedQueue::edgeQueueComparison(const cost_pair_t& lhs, const cost_pair_t& rhs) const
        {
            bool lhsLTrhs;

            //Get if LHS is less than RHS.
            lhsLTrhs = this->isCostBetterThan(lhs.first, rhs.first);

            //If it's not, it could be equal
            if (lhsLTrhs == false)
            {
                //If RHS is also NOT less than LHS, than they're equal and we need to check the second key
                if (this->isCostBetterThan(rhs.first, lhs.first) == false)
                {
                    //lhs == rhs
                    //Compare their second values
                    lhsLTrhs = this->isCostBetterThan( lhs.second, rhs.second );
                }
                //No else: lhs > rhs
            }
            //No else, lhs < rhs

            return lhsLTrhs;
        }



        bool IntegratedQueue::isCostBetterThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            return a.value() < b.value();
        }



        bool IntegratedQueue::isCostWorseThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If b is better than a, then a is worse than b
            return this->isCostBetterThan(b, a);
        }



        bool IntegratedQueue::isCostEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is not better than b, and b is not better than a, then they are equal
            return !this->isCostBetterThan(a,b) && !this->isCostBetterThan(b,a);
        }



        bool IntegratedQueue::isCostNotEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is better than b, or b is better than a, then they are not equal
            return this->isCostBetterThan(a,b) || this->isCostBetterThan(b,a);
        }



        bool IntegratedQueue::isCostBetterThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If b is not better than a, then a is better than, or equal to, b
            return !this->isCostBetterThan(b, a);
        }



        bool IntegratedQueue::isCostWorseThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is not better than b, than a is worse than, or equal to, b
            return !this->isCostBetterThan(a,b);
        }



        void IntegratedQueue::setUseFailureTracking(bool trackFailures)
        {
            useFailureTracking_ = trackFailures;
        }



        bool IntegratedQueue::getUseFailureTracking() const
        {
            return useFailureTracking_;
        }
    } // geometric
} //ompl
