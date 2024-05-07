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

#include <octomap/CostOcTree.h>

namespace octomap {

  // node implementation  --------------------------------------
  std::ostream& CostOcTreeNode::writeData(std::ostream &s) const {
    s.write((const char*) &occ_val, sizeof(occ_val)); // occupancy
    s.write((const char*) &cost, sizeof(double)); // cost

    return s;
  }

  std::istream& CostOcTreeNode::readData(std::istream &s) {
    s.read((char*) &occ_val, sizeof(occ_val)); // occupancy
    s.read((char*) &cost, sizeof(double)); // cost

    return s;
  }

  double CostOcTreeNode::getAverageChildCost() const {
    double sum = 0.0;
    int count = 0;

    if (children != NULL){
      for (int i = 0; i < 8; ++i) {
        CostOcTreeNode* child = static_cast<CostOcTreeNode*>(children[i]);
        if (child != NULL) {
          sum += child->getCost();
          ++count;
        }
      }
    }

    if (count > 0) {
      return sum / count;
    }
    else {
      return 0.0;
    }
  }

  void CostOcTreeNode::updateCostChildren() {
    cost = getAverageChildCost();
  }

  // tree implementation  --------------------------------------
  CostOcTree::CostOcTree(double in_resolution)
  : OccupancyOcTreeBase<CostOcTreeNode>(in_resolution) {
    costOcTreeMemberInit.ensureLinking();
  }

  CostOcTreeNode* CostOcTree::setNodeCost(const OcTreeKey& key, double c) {
    CostOcTreeNode* n = search(key);
    if (n != 0) {
      n->setCost(c);
    }
    return n;
  }

  bool CostOcTree::pruneNode(CostOcTreeNode* node) {
    // Add your implementation here
    // Example:
    // if (!isNodeCollapsible(node))
    //   return false;

    // Your pruning logic goes here

    return true; // Modify
  }

  bool CostOcTree::isNodeCollapsible(const CostOcTreeNode* node) const {
    // Add your implementation here
    // Example:
    // if (!nodeChildExists(node, 0))
    //   return false;

    // Your collapsibility check logic goes here

    return true; // Modify
  }

  CostOcTreeNode* CostOcTree::averageNodeCost(const OcTreeKey& key, double c) {
    CostOcTreeNode* n = search(key);
    if (n != 0) {
      n->setCost((n->getCost() + c) / 2.0);
    }
    return n;
  }

  CostOcTreeNode* CostOcTree::integrateNodeCost(const OcTreeKey& key, double c) {
    CostOcTreeNode* n = search(key);
    if (n != 0) {
      n->setCost(n->getCost() + c);
    }
    return n;
  }

  void CostOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void CostOcTree::updateInnerOccupancyRecurs(CostOcTreeNode* node, unsigned int depth) {
    // Add your implementation here
    // Example:
    // if (nodeHasChildren(node)){
    //   for (unsigned int i=0; i<8; i++) {
    //     if (nodeChildExists(node, i)) {
    //       updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
    //     }
    //   }
    //   node->updateOccupancyChildren();
    //   node->updateCostChildren();
    // }
  }

  CostOcTree::StaticMemberInitializer CostOcTree::costOcTreeMemberInit;

} // end namespace
