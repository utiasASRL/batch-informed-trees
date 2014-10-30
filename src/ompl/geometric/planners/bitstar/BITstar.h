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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_

//std::string
#include <string>
//std::pair
#include <utility>
//std::set and std::multiset
#include <set>
//boost::unordered_set (pre-C++11 std::unordered_set)
#include <boost/unordered_set.hpp>

//For boost::function
#include <boost/function.hpp>

//My vertex class:
#include "ompl/geometric/planners/bitstar/Vertex.h"
//My vertex-queue class
#include "ompl/geometric/planners/bitstar/VertexQueue.h"
//My edge-queue class
#include "ompl/geometric/planners/bitstar/EdgeQueue.h"
//The base-class of planners:
#include "ompl/base/Planner.h"
//The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"
//The informed sampler structure
#include "ompl/base/samplers/InformedStateSamplers.h"

//#include "ompl/geometric/planners/PlannerIncludes.h"

//#include <limits>
//#include <vector>
//#include <utility>


namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gBITTstar
            @par Short description
            \ref gBITstar "BIT*" (Batch Informed Trees) is an anytime asymptotically-optimal sampling-based
            motion planning algorithm that extends Lifelong Planning A* (LPA*) techniques to continuous planning
            problems. \ref gBITstar "BIT*" accomplishes this by processing batches of samples with a heuristic.
            In doing so, it strikes a balance between algorithms like RRT* and FMT*.

            @par J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs,"
            Submitted to ICRA 2015. <a href="http://arxiv.org/abs/1405.5848">arXiv:1405.5848 [cs.RO]</a>.
            <a href="http://www.youtube.com/watch?v=MRzSfLpNBmA">Illustration video</a>.

            @par TODO:
            - Make k-nearest correct.
            - Extend beyond single goal states to other samplable goals (i.e., goal sets).
            - Generalize heuristics to make proper use of the optimization class.
        */

        /** \brief Batch Informed Trees */
        class BITstar : public ompl::base::Planner
        {
        public:
            BITstar(const base::SpaceInformationPtr& si, const std::string& name = "BITstar");

            virtual ~BITstar();

            virtual void setup();

            virtual void clear();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Get the next edge to be processed. Causes a call to updateQueue and therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            std::pair<ompl::base::State*, ompl::base::State*> getNextEdgeInQueue();

            /** \brief Get the next edge actually in the queue. This may not be the next edge actually processed, as updateQueue may update the queue. */
            std::pair<ompl::base::State*, ompl::base::State*> getApproxNextEdgeInQueue() const;

            /** \brief Get the value of the next edge to be processed. Causes a call to updateQueue and therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            ompl::base::Cost getNextEdgeValueInQueue();

            /** \brief Get the value of the next edge actually in the queue.  This may not be the next edge actually processed, as updateQueue may update the queue. */
            ompl::base::Cost getApproxNextEdgeValueInQueue() const;

            /** \brief Get the whole messy queue. Expensive but helpful for some videos */
            void getQueue(std::vector<std::pair<VertexPtr, VertexPtr> >& edgesInQueue);


            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors();

            /** \brief Get the seed for the underlying StateSampler. Useful for running different settings with the exact same pseudorandom sequence. */
            boost::uint32_t getRngLocalSeed() const;

            /** \brief Set the seed for the underlying StateSampler. Useful for running different settings with the exact same pseudorandom sequence. */
            void setRngLocalSeed(boost::uint32_t seed);

            ///////////////////////////////////////
            // Planner settings:
            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Set the number of samplers per batch. */
            void setSamplesPerBatch(unsigned int n);

            /** \brief Get the number of samplers per batch. */
            unsigned int getSamplesPerBatch() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getKNearest() const;

            /** \brief Enable tracking of failed edges. This currently is too expensive to be useful.*/
            void setUseFailureTracking(bool trackFailures);

            /** \brief Get whether a failed edge list is in use.*/
            bool getUseFailureTracking() const;

            /** \brief Enable "strict sorting" of the edge queue.
            Rewirings can change the position in the queue of an edge.
            When strict sorting is enabled, the effected edges are resorted
            immediately, while disabling strict sorting delays this
            resorting until the end of the batch. */
            void setStrictQueueOrdering(bool beStrict);

            /** \brief Get whether strict queue ordering is in use*/
            bool getStrictQueueOrdering() const;

            /** \brief Enable pruning of vertices/samples that CANNOT improve the current solution.
            When a vertex in the graph is pruned, it's descendents are also pruned
            (if they also cannot improve the solution) or placed back in
            the set of free samples (if they could improve the solution).
            This assures that a uniform density is maintained.*/
            void setPruning(bool prune);

            /** \brief Get whether graph and sample pruning is in use.*/
            bool getPruning() const;

            /** \brief Stop the planner each time a solution improvement is found. Useful
            for examining the intermediate solutions found by BIT*. */
            void setStopOnSolnImprovement(bool stopOnChange);

            /** \brief Get whether BIT* stops each time a solution is found. */
            bool getStopOnSolnImprovement() const;
            ///////////////////////////////////////

            ///////////////////////////////////////
            // Planner progress property functions
            /** \brief Retrieve the best exact-solution cost found
            as a planner-progress property. */
            std::string bestCostProgressProperty() const;

            /** \brief Retrieve the number of batches processed
            as a planner-progress property. */
            std::string batchesProgressProperty() const;

            /** \brief Retrieve the number of iterations
            as a planner-progress property. */
            std::string iterationProgressProperty() const;

            /** \brief Retrieve the number of state collisions checks (i.e., calls to SpaceInformation::isValid(...))
            as a planner-progress property. */
            std::string stateCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of edge (or motion) collision checks (i.e., calls to SpaceInformation::checkMotion(...))
            as a planner-progress property. */
            std::string edgeCollisionCheckProgressProperty() const;

            /** \brief Retrieve the \e total number of samples generated
            as a planner-progress property. */
            std::string samplesGeneratedProgressProperty() const;

            /** \brief Retrieve the \e total number of vertices added to the graph
            as a planner-progress property. */
            std::string verticesConstructedProgressProperty() const;

            /** \brief Retrieve the number of nearest neighbour calls (i.e., NearestNeighbors<T>::nearestK(...) or NearestNeighbors<T>::nearestR(...))
            as a planner-progress property. */
            std::string nearestNeighbourProgressProperty() const;

            /** \brief Retrieve the number of edges that rewired the graph
            as a planner-progress property. */
            std::string rewiringProgressProperty() const;

            /** \brief Retrieve the current number of free samples
            as a planner-progress property. */
            std::string currentSampleProgressProperty() const;

            /** \brief Retrieve the current number of vertices in the graph
            as a planner-progress property. */
            std::string currentVertexProgressProperty() const;

            /** \brief Retrieve the current number of edges in the edge queue
            as a planner-progress property. */
            std::string queueSizeProgressProperty() const;
            ///////////////////////////////////////

        protected:

        private:
            //Typedefs:
            typedef std::pair<VertexPtr, VertexPtr> vertex_pair_t;

            //Functions:
            /** \brief A debug function: Estimate the measure of the free/obstace space via sampling. */
            void estimateMeasures();

            ///////////////////////////////////////////////////////////////////
            //BIT* primitives:
            /** \brief Initialize variables for a new batch */
            void newBatch();

            /** \brief Make sure that all vertices in our tree with a cost-to-come less than the minimum cost in our edge queue has been expanded. Finds all potential edges from the vertices to nearby free states and adds those edges to the queue. */
            void updateQueue();

            /** \brief Update the edge queue by adding all the potential edges from the vertex to nearby free states */
            void expandVertex(const VertexPtr& vertex);

            /** \brief Update the list of free samples */
            void updateSamples(const VertexPtr& vertex);

            /** \brief Prune all samples with a solution heuristic that is not less than the bestCost_ */
            void pruneSamples();

            /** \brief Prune all vertices with a solution heuristic that is greater than the bestCost_ */
            void pruneGraph();

//            /** \brief Prune all failed edges where the parent vertex is not disconnected. */
//            void pruneFailedEdgeSet();

            /** \brief Publish the found solution to the ProblemDefinition*/
            void publishSolution();
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for data manipulation
//            /** \brief Free the memory for the non shared pointer version */
//            void freeMemory();

            /** \brief Prune a vertex, recursively pruning its children and either (a) deleting them (b) placing them back as disconnected states depending on their heuristic value compared to the best solution*/
            void pruneVertex(const VertexPtr& oldVertex);

            /** \brief Actually remove a vertex from the graph */
            void removeVertex(const VertexPtr& oldVertex, bool recycleVertex);

            /** \brief Disconnect a vertex, removing it from the graph but not touching the edge queue. */
            void disconnectVertex(const VertexPtr& oldVertex);

            /** \brief Add an edge from the edge queue to the tree. Will add the state to the vertex queue if it's new to the tree or otherwise replace the parent. */
            void addEdge(const vertex_pair_t& newEdge, const ompl::base::Cost& edgeCost);

            /** \brief Replace the parent edge with the given new edge and cost */
            void replaceParent(const vertex_pair_t& newEdge, const ompl::base::Cost& edgeCost);

            /** \brief Add a sample */
            void addSample(const VertexPtr& newSample);

            /** \brief Add a vertex to the graph */
            void addVertex(const VertexPtr& newVertex);

            /** \brief Attempt to add an edge to the queue. Checks that the edge meets the queue condition and that it is not in the failed set. */
            bool queueupEdge(const VertexPtr& parent, const VertexPtr& child);
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for sorting queues/nearest-neighbour structures and the related calculations. Some of these should probably be generalized into OptimizationObjective?
            typedef std::pair<ompl::base::Cost, ompl::base::Cost> cost_pair_t;

            /** \brief The distance function used for nearest neighbours. Calculates the distance directionally from the given state to all the other states (can be used on states either in our out of the graph).*/
            double nnDistance(const VertexPtr& a, const VertexPtr& b) const;

            /** \brief A convenience function for the value of a vertex in the queue, uses currentHeuristicVertex */
            ompl::base::Cost vertexQueueValue(const VertexPtr& vertex) const;

            /** \brief A convenience function for the value of an edge in the queue, uses currentHeuristicEdge */
            cost_pair_t edgeQueueValue(const vertex_pair_t& edge) const;

            /** \brief The comparison function for two vertices in the expansion queue. Uses vertexQueueValue. */
            bool vertexQueueComparison(const ompl::base::Cost& lhs, const ompl::base::Cost& rhs) const;

            /** A comparison function for two edges (i.e., vertex-pairs) in the graph. Uses edgeQueueValue. */
            bool edgeQueueComparison(const cost_pair_t& lhs, const cost_pair_t& rhs) const;

            /** The condition for a sample to be in the sample list. Compares lowerBoundHeuristicVertex to the given threshold. Returns true if the vertex's best cost is less than the threshold. */
            bool sampleQueueCondition(const VertexPtr& state, const ompl::base::Cost& threshold) const;

            /** The condition for a vertex to be in the state list. Compares lowerBoundHeuristicVertex to the given threshold. Returns true if the vertex's best cost is less than or equal to the threshold. */
            bool vertexQueueCondition(const VertexPtr& state, const ompl::base::Cost& threshold) const;

            /** The condition for an edge (i.e., vertex-pair) to be in the edge queue. Compares lowerBoundHeuristicEdge to the given threshold. Returns true if the edge's best cost is less than the threshold. */
            bool edgeQueueCondition(const vertex_pair_t& edge, const ompl::base::Cost& threshold) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for various heuristics.
            /** \brief Calculates the heuristic estimate of a cost constrained to pass through a vertex, independent of the current cost-to-come. I.e., combines the heuristic estimates of the cost-to-come and cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicVertex(const VertexPtr& edgePair) const;

            /** \brief Calculates the heuristic estimate of a cost constrained to pass through a vertex, dependent on the current cost-to-come. I.e., combines the current cost-to-come with a heuristic estimate of the cost-to-go. */
            ompl::base::Cost currentHeuristicVertex(const VertexPtr& edgePair) const;

            /** \brief Calculates the heuristic estimate of a cost constrained to go through an edge, independent of the cost-to-come of the parent state. I.e., combines the heuristic estimates of the cost-to-come, edge cost, and cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicEdge(const vertex_pair_t& edgePair) const;

            /** \brief Calculates the heuristic estimate of a cost constrained to go through an edge, dependent on the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic estimates of the edge cost, and cost-to-go. */
            ompl::base::Cost currentHeuristicEdge(const vertex_pair_t& edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-come for a Vertex */
            ompl::base::Cost costToComeHeuristic(const VertexPtr& vertex) const;

            /** \brief Calculate a heuristic estimate of the cost an edge between two Vertices */
            ompl::base::Cost edgeCostHeuristic(const vertex_pair_t& edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-go for a Vertex */
            ompl::base::Cost costToGoHeuristic(const VertexPtr& vertex) const;

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. I.e., For path-length problems, the cost equivalent of +2*r. */
            ompl::base::Cost neighbourhoodCost() const;

            /** \brief The true cost of an edge, including collisions.*/
            ompl::base::Cost trueEdgeCost(const vertex_pair_t& edgePair) const;

            /** \brief Combine three costs as (a + b) + c */
            ompl::base::Cost combineCosts(const ompl::base::Cost& a, const ompl::base::Cost& b, const ompl::base::Cost& c) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions to calculate parameters:
            /** \brief Get the nearest samples from the freeStateNN_ using the appropriate "near" definition (i.e., k or r). */
            void nearestSamples(const VertexPtr& vertex, std::vector<VertexPtr>* neighbourSamples);

            /** \brief Get the nearest samples from the vertexNN_ using the appropriate "near" definition (i.e., k or r). */
            void nearestVertices(const VertexPtr& vertex, std::vector<VertexPtr>* neighbourVertices);

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_ */
            void updateNearestTerms(unsigned int N);

            /** \brief Calculate the r for r-disc nearest neighbours, a function of the current graph */
            double r(unsigned int N) const;

            /** \brief Calculate the k for k-nearest neighours, a function of the current graph */
            unsigned int k(unsigned int N) const;

            /** \brief Calculate the lower-bounding radius RGG term for asymptotic almost-sure convergence to the optimal path (i.e., r_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the size of the problem domain. */
            double minimumRggR() const;

            /** \brief Calculate the lower-bounding k-nearest RGG term for asymptotic almost-sure convergence to the optimal path (i.e., k_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the state dimension and is left as a double for later accuracy in calculate k */
            double minimumRggK() const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper function for debug logging
            void statusMessage(const ompl::msg::LogLevel& msgLevel, const std::string& status) const;
            ///////////////////////////////////////////////////////////////////



            //Variables -- Make sure everyone is configured in setup() and reset in clear():
            /** \brief State sampler */
            ompl::base::InformedStateSamplerPtr                      sampler_;

            /** \brief Optimization objective copied from ProblemDefinition */
            ompl::base::OptimizationObjectivePtr                     opt_;

            /** \brief The start of the problem as a vertex*/
            VertexPtr                                                startVertex_;

            /** \brief The goal of the problem as a vertex*/
            VertexPtr                                                goalVertex_;

            /** \brief The unconnected samples as a nearest-neighbours datastructure. Sorted by nnDistance. */
            boost::shared_ptr< NearestNeighbors<VertexPtr> >         freeStateNN_;

            /** \brief The vertices as a nearest-neighbours data structure. Sorted by nnDistance. */
            boost::shared_ptr< NearestNeighbors<VertexPtr> >         vertexNN_;

            /** \brief The planning graph as a queue sorted on f-value (total heuristic cost). Sorted by vertexComparison. */
            VertexQueue                                              vertexQueue_;

            /** \brief The queue of new edges to process. Sorted by edgeComparison. */
            EdgeQueue                                                edgeQueue_;

//            /** \brief The set of edges that have been previously tried (i.e., had their true cost calculated) and failed. */
//            boost::unordered_set<vertex_pair_t>                      failedEdgeSet_;

            ///////////////////////////////////////
            //Parameters
            /** \brief Whether to use a strict-queue ordering (param) */
            bool                                                     useStrictQueueOrdering_;

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (param) */
            double                                                   rewireFactor_;

            /** \brief The number of samples per batch (param) */
            unsigned int                                             samplesPerBatch_;

            /** \brief Track edges that have been checked and failed so they never reenter the queue. (param) */
            bool                                           useFailureTracking_;

            /** \brief Option to use k-nearest search for rewiring (param) */
            bool                                           useKNearest_;

            /** Whether to use graph pruning (param) */
            bool                                                     usePruning_;

            /** Whether to stop the planner as soon as the path changes (param) */
            bool                                                     stopOnSolnChange_;
            ///////////////////////////////////////

            /** \brief The resulting sampling density for a batch */
            double                                                   sampleDensity_;

            /** \brief The current r-disc RGG connection radius */
            double                                                   r_;

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be calculated once. Left as a double for later accuracy in calculate k*/
            double                                            k_rgg_;

            /** \brief The current k-nearest RGG connection number*/
            unsigned int                                            k_;

            /** \brief The best cost found to date. This is the maximum total-heuristic cost of samples we'll consider. */
            ompl::base::Cost                                         bestCost_;

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the goal. */
            ompl::base::Cost                                         minCost_;

            /** \brief The total-heuristic cost up to which we've sampled */
            ompl::base::Cost                                         costSampled_;

            /** \brief If we've found a solution */
            bool                                                     hasSolution_;

            /** \brief If the solution is approximate */
            bool                                                     approximateSoln_;

            /** \brief The distance of the approximate solution, set to -1.0 for non approximate solutions */
            double                                                   approximateDiff_;

            /** \brief The number of iterations run */
            unsigned int                                             numIterations_;

            /** \brief The total number of samples generated */
            unsigned int                                             numSamples_;

            /** \brief The number of vertices ever added to the graph. Will count vertices twice if they spend anytime disconnected */
            unsigned int                                             numVertices_;

            /** \brief The number of state collision checks */
            unsigned int                                             numStateCollisionChecks_;

            /** \brief The number of edge collision checks */
            unsigned int                                             numEdgeCollisionChecks_;

            /** \brief The number of nearest neighbour calls */
            unsigned int                                             numNearestNeighbours_;

            /** \brief The number of times a state in the graph was rewired */
            unsigned int                                             numRewirings_;

            /** \brief The number of batches processed */
            unsigned int                                             numBatches_;
        }; //class: BITstar
    } //geometric
} //ompl
#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_
