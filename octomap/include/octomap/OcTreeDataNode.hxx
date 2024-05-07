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

namespace octomap {

  template <typename T, typename U>
  OcTreeDataNode<T, U>::OcTreeDataNode()
   : children(NULL)
  {

  }

  template <typename T, typename U>
  OcTreeDataNode<T, U>::OcTreeDataNode(T initOccVal, U initCostVal)
   : children(NULL), occ_val(initOccVal), cost_val(initCostVal)
  {

  }

  template <typename T, typename U>
  OcTreeDataNode<T, U>::OcTreeDataNode(const OcTreeDataNode<T, U>& rhs)
   : children(NULL), occ_val(rhs.occ_val), cost_val(rhs.cost_val)
  {
    if (rhs.children != NULL){
      allocChildren();
      for (unsigned i = 0; i<8; ++i){
        if (rhs.children[i] != NULL)
          children[i] = new OcTreeDataNode<T, U>(*(static_cast<OcTreeDataNode<T, U>*>(rhs.children[i])));

      }
    }
  }
  
  template <typename T, typename U>
  OcTreeDataNode<T, U>::~OcTreeDataNode()
  {
    // Delete only own members. OcTree maintains tree structure and must have deleted 
    // children already
    assert(children == NULL);
  }
  
  template <typename T, typename U>
  void OcTreeDataNode<T, U>::copyData(const OcTreeDataNode<T, U>& from){
    occ_val = from.occ_val;
    cost_val = from.cost_val;
  }

  template <typename T, typename U>
  bool OcTreeDataNode<T, U>::operator== (const OcTreeDataNode<T, U>& rhs) const{
    // rhs.cost_val == cost_val
    // Since costs have no min_max clamps, equality check should probably be performed
    // using a custom doubleEquals(a, b, epsilon) function instead of ==

    // But == might work too depending on the symmetricity of costs. TODO
    return rhs.occ_val == occ_val && rhs.cost_val == cost_val;
  }

  // ============================================================
  // =  children          =======================================
  // ============================================================


  template <typename T, typename U>
  bool OcTreeDataNode<T, U>::childExists(unsigned int i) const {
    assert(i < 8);
    if ((children != NULL) && (children[i] != NULL))
      return true;
    else
      return false;
  }
  
  template <typename T, typename U>
  bool OcTreeDataNode<T, U>::hasChildren() const {
    if (children == NULL)
      return false;
    for (unsigned int i = 0; i<8; i++){
      // fast check, we know children != NULL
      if (children[i] != NULL)
        return true;
    }
    return false;
  }


  // ============================================================
  // =  File IO           =======================================
  // ============================================================

  template <typename T, typename U>
  std::istream& OcTreeDataNode<T, U>::readData(std::istream &s) {
    s.read((char*) &occ_val, sizeof(occ_val));
    s.read((char*) &cost_val, sizeof(cost_val));
    return s;
  }


  template <typename T, typename U>
  std::ostream& OcTreeDataNode<T, U>::writeData(std::ostream &s) const{
    s.write((const char*) &occ_val, sizeof(occ_val));
    s.write((const char*) &cost_val, sizeof(cost_val));
    return s;
  }


  // ============================================================
  // =  private methodes  =======================================
  // ============================================================
  template <typename T, typename U>
  void OcTreeDataNode<T, U>::allocChildren() {
    children = new AbstractOcTreeNode*[8];
    for (unsigned int i=0; i<8; i++) {
      children[i] = NULL;
    }
  }


} // end namespace

