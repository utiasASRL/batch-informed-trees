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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_VERTEXQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_VERTEXQUEUE_

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

        /** \brief A vertex queue. */
        class VertexQueue
        {
        public:
            /** \brief A boost::function definition for the value function of the queue. Given an edge, return the cost pair used to sort in the queue. */
            typedef boost::function<ompl::base::Cost (const VertexPtr&)> value_func_t;

            /** \brief A boost::function definition for the sorting function of the queue. Given two vertices, a, b, return true if a comes before b in the desired ordering, i.e., a < b? */
            typedef boost::function<bool (const ompl::base::Cost&, const ompl::base::Cost&)> sorting_func_t;

            /** \brief A boost::function definition for the boolean requirement function for the queue. Given a vertex, a, and a cost, t, return true if the vertex should be in the queue, i.e., a <= t? */
            typedef boost::function<bool (const VertexPtr&, const ompl::base::Cost&)> threshhold_func_t;

            /** \brief A typedef to the underlying queue as a multiset */
            typedef std::multimap<ompl::base::Cost, VertexPtr, sorting_func_t> cost_vertex_multimap_t;

            /** \brief Construct a vertex queue. */
            VertexQueue(const value_func_t& valueFunc, const sorting_func_t& sortFunc, const threshhold_func_t& threshFunc);

            /** \brief Insert a vertex into the vertex queue */
            void insert(const VertexPtr& newVertex);

            /** \brief Get the best vertex on the queue, leaving it on the queue. */
            VertexPtr front() const;

            /** \brief Get the value of the best vertex on the queue, leaving it on the queue */
            ompl::base::Cost front_value() const;

            /** \brief Pop the best vertex  off the queue, removing it from the queue in the process. */
            void pop_front(VertexPtr& bestVertex);

            /** \brief Pop the best vertex  off the queue, removing it from the queue in the process. */
            VertexPtr pop_front();

            /** \brief Get a copy of the full queue. This is expensive. */
            void list(std::vector<VertexPtr>& fullQueue);

            /** \brief Resort the queue, only reinserting vertices if their lower-bound heuristic is less then the given threshold. Requires first marking the queue as unsorted */
            void resort(const ompl::base::Cost& costThreshold);

            /** \brief Erase a vertex if it exists in the queue */
            void erase(const VertexPtr& oldVertex);

            /** \brief Clear the queue */
            void clear();

            /** \brief Returns true if the queue is empty */
            bool empty() const;

            /** \brief Returns the number of elements in the queue */
            unsigned int size() const;

            /** \brief Return whether the queue is still sorted */
            bool isSorted() const;

            /** \brief Mark the queue as requiring resorting */
            void markUnsorted(const VertexPtr& vertex);

        protected:

        private:
            /** \brief A typedef for an iterator into the multimap */
            typedef cost_vertex_multimap_t::iterator mmap_iter_t;

            /** \brief A typedef to an unordered_map of multimap iterators */
            typedef boost::unordered_map<VertexPtr, mmap_iter_t> vertex_iter_umap_t;

            ////////////////////////////////
            //Member variables:
            /** \brief The value function used by the queue */
            value_func_t valueFunc_;

            /** \brief The sorting function used by the queue */
            sorting_func_t sortFunc_;

            /** \brief The threshold function used by the queue */
            threshhold_func_t threshFunc_;

            /** \brief The underlying queue of vertices. Sorted by sorting_func_t. */
            cost_vertex_multimap_t                                  vertexQueue_;

            /** \brief A lookup from vertex to map iter */
            vertex_iter_umap_t vertexIterLookup_;

            /** \brief A list of vertices that we will need to process when resorting the queue: */
            std::list<VertexPtr>           resortVertices_;
            ////////////////////////////////

            ////////////////////////////////
            //Helper functions:
            /** \brief Insert a vertex into the edge queue and lookup given as a vertex and an optional hint. */
            void insert_helper(const VertexPtr& newEdge, mmap_iter_t* positionHint = NULL);
            ////////////////////////////////

        }; //class: VertexQueue
    } //geometric
} //ompl
#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_VERTEXQUEUE_

