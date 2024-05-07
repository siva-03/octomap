/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_OCTREE_NODE_H
#define OCTOMAP_OCTREE_NODE_H

#include "octomap_types.h"
#include "octomap_utils.h"
#include "OcTreeDataNode.h"
#include <limits>

namespace octomap {

  /**
   * Nodes to be used in OcTree. They represent 3d occupancy grid cells.
   * "value" stores their log-odds occupancy.
   *
   * Note: If you derive a class (directly or indirectly) from OcTreeNode or 
   * OcTreeDataNode, you have to implement (at least) the following functions:
   * createChild(), getChild(), getChild() const, expandNode() to avoid slicing
   * errors and memory-related bugs.
   * See ColorOcTreeNode in ColorOcTree.h for an example.
   *
   */
  class OcTreeNode : public OcTreeDataNode<float, double> {

  public:
    OcTreeNode();
    ~OcTreeNode();

    
    // -- node occupancy  ----------------------------

    /// \return occupancy probability of node
    inline double getOccupancy() const { return probability(occ_val); }

    /// \return cost of a node
    inline double getCost() const { return cost_val; }

    /// \return log odds representation of occupancy probability of node
    inline float getLogOdds() const{ return occ_val; }
    /// sets log odds occupancy of node
    inline void setLogOdds(float l) { occ_val = l; }
    /// sets cost of node
    inline void setCost(double l) { cost_val = l; }

    /**
     * @return mean of all children's occupancy probabilities, in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return maximum of children's occupancy probabilities, in log odds
     */
    float getMaxChildLogOdds() const;

    /**
     * @return mean of all children's costs
     */
    double getMeanChildCosts() const;

    /**
     * @return maximum of children's costs
     */
    double getMaxChildCosts() const;

    /**
     * @return total of children's costs
     */
    double getTotalChildCosts() const;

    /// update this node's occupancy according to its children's maximum occupancy
    inline void updateOccupancyChildren() {
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative occ
    }

    /// update this node's cost according to its children's (max or total)? cost
    inline void updateCostChildren() {
      this->setCost(this->getTotalChildCosts());  // putting total as of now - change as necessary TODO
    }

    /// adds p to the node's logOdds value (with no boundary / threshold checking!)
    void addOccValue(const float& p);

    /// adds c to the node's cost value (with no boundary / threshold checking!)
    /// no sure if this is really necessary
    void addCostValue(const double& c);
    
    void setCostValue(const double& c);

  protected:
    // "value" stores log odds occupancy probability
  };

} // end namespace

#endif
