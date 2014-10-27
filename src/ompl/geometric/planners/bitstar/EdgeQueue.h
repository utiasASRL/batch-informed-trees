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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_EDGEQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_EDGEQUEUE_

//std::pair
#include <utility>
//std::list
#include <list>
//std::multimap
#include <map>
//boost::unordered_set (pre-C++11 std::unordered_set)
#include <boost/unordered_map.hpp>

//For boost::function
#include <boost/function.hpp>
//The vertex class:
#include "ompl/geometric/planners/bitstar/Vertex.h"
#include <ompl/util/Time.h>

namespace ompl
{
    namespace geometric
    {

        /** \brief An edge queue. */
        class EdgeQueue
        {
        public:
            /** \brief A typedef for a pair of vertices, i.e., an edge */
            typedef std::pair<VertexPtr, VertexPtr> vertex_pair_t;

            /** \brief A typedef for a pair of costs, i.e., the sorting key */
            typedef std::pair<ompl::base::Cost, ompl::base::Cost> cost_pair_t;

            /** \brief A boost::function definition for the value function of the queue. Given an edge, return the cost pair used to sort in the queue. */
            typedef boost::function<cost_pair_t (const vertex_pair_t&)> value_func_t;

            /** \brief A boost::function definition for the sorting function of the queue. Given two cost pairs, a, b, return true if a comes before b in the desired ordering, i.e., a < b? */
            typedef boost::function<bool (const cost_pair_t&, const cost_pair_t&)> sorting_func_t;

            /** \brief A boost::function definition for the boolean requirement function for the queue. Given an edge, a, and a cost, t, return true if the edge should be in the queue, i.e., a <= t? */
            typedef boost::function<bool (const vertex_pair_t&, const ompl::base::Cost&)> threshhold_func_t;

            /** \brief A typedef to the underlying queue as a multimap. The advantage to a multimap over a multiset is that a copy of the key is stored with the value, which guarantees that the ordering remains sane. Even if the inherent key for a value has changed, it will still be sorted under the old key until manually updated and the map will be sorted */
            typedef std::multimap<cost_pair_t, vertex_pair_t, sorting_func_t> cost_pair_vertex_pair_multimap_t;

            /** \brief Construct an edge queue. */
            EdgeQueue(bool useParentVertexLookupTables, bool useChildVertexLookupTables, const value_func_t& valueFunc, const sorting_func_t& sortFunc, const threshhold_func_t& threshFunc);

            /** \brief Insert an edge into the edge queue given as a an edge pair */
            void insert(const vertex_pair_t& newEdge);

            /** \brief Insert an edge into the edge queue given as a parent (p) and child (c) vertex*/
            void insert(const VertexPtr& pVertex, const VertexPtr& cVertex);

            /** \brief Erase all edges in the edge queue that lead to the given vertex */
            void remove_to(const VertexPtr& cVertex);

            /** \brief Erase all edges in the edge queue that leave from the given vertex */
            void remove_from(const VertexPtr& pVertex);

            /** \brief Prune all edges in the edge queue that lead to the given vertex using the given prune function  */
            void prune_to(const VertexPtr& cVertex, const ompl::base::Cost& costThreshold);

            /** \brief Prune all edges in the edge queue that leave from the given vertex using the given prune function */
            void prune_from(const VertexPtr& pVertex, const ompl::base::Cost& costThreshold);

            /** \brief Get the best edge on the queue, leaving it on the queue. */
            vertex_pair_t front() const;

            /** \brief Get the value of the best edge on the queue, leaving it on the queue */
            cost_pair_t front_value() const;

            /** \brief Pop the best edge off the queue, removing it from the queue in the process. */
            void pop_front(vertex_pair_t& bestEdge);

            /** \brief Pop the best edge off the queue, removing it from the queue in the process. */
            vertex_pair_t pop_front();

            /** \brief Get a copy of the full queue. This is expensive. */
            void list(std::vector<vertex_pair_t>& fullQueue);

//            /** \brief Flag a vertex as requiring queue resorting for all descendent edges in the queue*/
//            void flagVertex(VertexPtr vertex);

            /** \brief Resort the queue, only reinserting edges if their lower-bound heuristic is less then the given threshold. As "vertex flagging" is not implemented, requires first marking the queue as unsorted */
            void resort(const ompl::base::Cost& costThreshold);
//            ompl::time::duration resort(const ompl::base::Cost& costThreshold);

            /** \brief Clear the queue */
            void clear();

            /** \brief Returns true if the queue is empty */
            bool empty() const;

            /** \brief Returns the number of elements in the queue */
            unsigned int size() const;

            /** \brief Get the number of edges in the queue pointing to a specific vertex */
            unsigned int num_to(const VertexPtr& cVertex) const;

            /** \brief Get the number of edges in the queue coming from a specific vertex */
            unsigned int num_from(const VertexPtr& pVertex) const;


            /** \brief Return whether the queue is still sorted */
            bool isSorted() const;

            /** \brief Mark the queue as requiring resorting as a result of the given vertex */
            void markUnsorted(const VertexPtr& vertex);

        protected:

        private:
            /** \brief A typedef for an iterator into the multimap */
            typedef cost_pair_vertex_pair_multimap_t::iterator mmap_iter_t;

            /** \brief A typedef for a list of multimap iterators*/
            typedef std::list<mmap_iter_t> mmap_iter_list_t;

            /** \brief A typedef to an unordered_map of multimap iterators */
            typedef boost::unordered_map<VertexPtr, mmap_iter_list_t > vertex_iter_umap_t;

            ////////////////////////////////
            //Member variables:
            /** \brief The value function used by the queue */
            value_func_t valueFunc_;

            /** \brief The sorting function used by the queue */
            sorting_func_t sortFunc_;

            /** \brief The threshold function used by the queue */
            threshhold_func_t threshFunc_;

            /** \brief Whether to use parent lookup tables or not */
            bool outgoingLookupTables_;

            /** \brief Whether to use child lookup tables or not */
            bool incomingLookupTables_;

            /** \brief The underlying queue of edges. Sorted by sorting_func_t. */
            cost_pair_vertex_pair_multimap_t                                  edgeQueue_;

            /** \brief A unordered map from a vertex to all the edges in the queue emanating from the vertex: */
            vertex_iter_umap_t                      outgoingEdges_;

            /** \brief A unordered map from a vertex to all the edges in the queue leading into the vertex: */
            vertex_iter_umap_t                      incomingEdges_;

            /** \brief A list of vertices that we will need to process when resorting the queue: */
            std::list<VertexPtr>           resortVertices_;
            ////////////////////////////////

            ////////////////////////////////
            //Helper functions:
            /** \brief Insert an edge into the edge queue and lookups given as a an edge pair and an optional hint. */
            void insert_helper(const vertex_pair_t& newEdge, mmap_iter_t* positionHint = NULL);

            /** \brief Erase a lookup value of an iterator*/
            void eraseLookups(const mmap_iter_t& iterToRmFrmLookup);

            /** \brief Helper wrapper to remove an incoming lookup*/
            void rmIncomingLookup(const mmap_iter_t& mmapIterToRm);

            /** \brief Helper wrapper to remove an outgoing lookup*/
            void rmOutgoingLookup(const mmap_iter_t& mmapIterToRm);

            /** \brief Erase an edge from the given lookup container at the specified index */
            void rmLookup(vertex_iter_umap_t& lookup, const VertexPtr& idx, const mmap_iter_t& mmapIterToRm);
            ////////////////////////////////

        }; //class: EdgeQueue
    } //geometric
} //ompl
#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_EDGEQUEUE_

