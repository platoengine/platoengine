"""

//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
"""
    
"""
high level wrapper for the Exodus-Python interface, expy.
"""
import expy

class ElementBlock:
  "Exodus-like element block"

  def __init__(self):
    self.connectivity = []
    self.elemType = "Unset"
    self.numDimensions = 0
    self.Id = 0
    self.numElements = 0
    self.numNodesPerElement = 0
    self.numAttr = 0
    self.coordinates = []
    self.ElemTriMap = []
    self.ElemVolumes = []
    self.name = ""


  def Initialize(self, blk_id, elem_type, num_dim, num_elem, num_nodes, num_attr, connect, coord):
    self.elemType = elem_type
    self.numDimensions = num_dim
    self.numElements = num_elem
    self.numNodesPerElement = num_nodes
    self.numAttr = num_attr
    self.connectivity = connect
    self.Id = blk_id
    self.coordinates = coord

  def Connectivity(self, block_local_elem_id):
    """ 
    Takes local element index, returns connectivity as int list

    Arguments:
    block_local_elem_id -- element index in block local numbering,
                           i.e., 0<n<Nel_this_block

    Returns: node indices of comprising nodes.  See Exodus API
             for local numbering convention.
    """
    start = self.numNodesPerElement* block_local_elem_id
    stop = start + self.numNodesPerElement;
    if start >= 0 & stop < len(self.connectivity):
      return [self.connectivity[i] for i in range(start,stop)]
    else:
      return

  def getElement(self, block_local_elem_id):
    """ 
    Takes local element index, returns array of nodal positions

    Arguments:
    block_local_elem_id -- element index in block local numbering,
                           i.e., 0<n<Nel_this_block

    Returns: locations of comprising nodes.  See Exodus API
             for local numbering convention.
    """
    X = self.coordinates[0]
    Y = self.coordinates[1]
    if self.numDimensions == 3:
      Z = self.coordinates[2]
    conn = self.Connectivity(block_local_elem_id)
    if self.numDimensions == 2:
      elempoints = [[X[node],Y[node]] for node in conn]
    if self.numDimensions == 3:
      elempoints = [[X[node],Y[node],Z[node]] for node in conn]
    return elempoints

class NodeSet:
  "Exodus-style node set"
  def __init__(self, ns_id, num_nodes, num_dists, nodes, name=""):
    self.Id = ns_id
    self.numNodes = num_nodes
    self.numDists = num_dists
    self.nodes = nodes
    self.name = name if (name!="") else ("unnamed_ns_" + str(ns_id))

class SideSet:
  "Exodus-style side set"
  def __init__(self, ss_id, num_sides, num_dists, elems, sides, name=""):
    self.Id = ss_id
    self.numSides = num_sides
    self.numDists = num_dists
    self.elems = elems
    self.sides = sides
    self.name = name if (name!="") else ("unnamed_ss_" + str(ss_id))


class ExodusDB:
  "Exodus database object"
  def __init__(self):
    self.inFileName = ""
    self.outFileName = ""
    self.iMode = ""
    self.compWordSize = 8
    self.ioWordSize = 8
    self.inFileId = -1

    self.title = "not set"
    self.numDimensions = 0
    self.numNodes = 0
    self.numElements = 0

    self.elementBlocks = []

    self.numNodeSets = 0
    self.NodeSets = []

    self.numSideSets = 0
    self.SideSets = []

    self.coordinateNames = []
    self.coordinates = []

    self.numElemVars = 0
    self.elemVarNames = []
    self.elemVars = []

    self.numNodeVars = 0
    self.nodeVarNames = []
    self.nodeVars = []

    self.varTimes = []
    self.nodeMap = []
    self.elemMap = []

    # elemAttr is size [num_blocks][num_attr*num_elem_this_blk]
    self.elemAttr = []

    self.comp_ws = 8
    self.io_ws = 8

  def getCoordData(self, node_id, inputType="id"):

    if inputType == "id":
      if len(self.nodeMap) > 0:
        if self.nodeMap.count(node_id) == 0:
          print "node id ", node_id, " not found"
          return 0
        else:
          node_index = self.nodeMap.index(node_id)
      else:
        print "node map not found"
    else:
      node_index = node_id

    if self.numDimensions == 2:
      return [self.coordinates[0][node_index],self.coordinates[1][node_index]]
    elif self.numDimensions == 3:
      return [self.coordinates[0][node_index],self.coordinates[1][node_index],self.coordinates[2][node_index]]

  def Connectivity(self, elem_gid):

    if self.elemMap.count(elem_gid) == 0:
      print "elem id ", elem_gid, " not found"
      return 0
    else:
      elem_index = self.elemMap.index(elem_gid)

    block_index = 0
    for block in self.elementBlocks:
      elem_index -= block.numElements
      block_index += 1
      if elem_index <  0:
        elem_index += block.numElements
        block_index -= 1
        break

    return self.elementBlocks[block_index].Connectivity(elem_index)
    

  def getElemDataBE(self, time_step, var_name, block_id, elem_lid):

    if time_step >= len(self.varTimes):
      print "time_step doesn't exist"
      return 0
    if self.elemVarNames.count(var_name) != 1:
      print "Variable not found"
      return 0
    else:
      var_index = self.elemVarNames.index(var_name)

    if len(self.elemVars[time_step][var_index]) > 0:
      return self.elemVars[time_step][var_index][block_id][elem_lid]
    else:
      for block in self.elementBlocks:
        self.elemVars[time_step][var_index].append( \
           expy.get_elem_var(self.inFileId, time_step+1, var_index+1, block.Id, block.numElements ))
      return self.elemVars[time_step][var_index][block_id][elem_lid]

  def getElemData(self, time_step, var_name, elem_gid=-1):

    if time_step >= len(self.varTimes):
      print "time_step doesn't exist"
      return 0
    if self.elemVarNames.count(var_name) != 1:
      print "Variable not found"
      return 0
    else:
      var_index = self.elemVarNames.index(var_name)

    # find in the element map
    if elem_gid >= 0:
      if self.elemMap.count(elem_gid) == 0:
        print "elem id ", elem_gid, " not found"
        return 0
      else:
        elem_index = self.elemMap.index(elem_gid)

      # find elem lid
      block_index = 0
      for block in self.elementBlocks:
        elem_index -= block.numElements
        block_index += 1
        if elem_index < 0:
          elem_index += block.numElements
          block_index -= 1
          break
      if len(self.elemVars[time_step][var_index]) > 0:
        return self.elemVars[time_step][var_index][block_index][elem_index]
      else:
        for block in self.elementBlocks:
          self.elemVars[time_step][var_index].append( expy.get_elem_var(self.inFileId, time_step+1, var_index+1, block.Id, block.numElements ))
        return self.elemVars[time_step][var_index][block_index][elem_index]

    else:
      if len(self.elemVars[time_step][var_index]) > 0:
        return self.elemVars[time_step][var_index]
      else:
        for block in self.elementBlocks:
          self.elemVars[time_step][var_index].append( expy.get_elem_var(self.inFileId, time_step+1, var_index+1, block.Id, block.numElements ))
        return self.elemVars[time_step][var_index]
      
  def getNodeData(self, time_step, var_name, node_id=-1):
    if time_step >= len(self.varTimes):
      print "time_step doesn't exist"
      return 0
    if self.nodeVarNames.count(var_name) != 1:
      print "Variable not found"
      return 0
    else:
      var_index = self.nodeVarNames.index(var_name)
      
    if node_id >= 0:
      if self.nodeMap.count(node_id) == 0:
        print "node id ", node_id, " not found"
        return 0
      else:
        node_index = self.nodeMap.index(node_id)
  
      if len(self.nodeVars[time_step][var_index]) > 0:
        return self.nodeVars[time_step][var_index][node_index]
      else:
        self.nodeVars[time_step][var_index] = expy.get_nodal_var(self.inFileId, time_step+1, var_index+1, self.numNodes)
        return self.nodeVars[time_step][var_index][node_index]
    else:
      if len(self.nodeVars[time_step][var_index]) > 0:
        return self.nodeVars[time_step][var_index]
      else:
        self.nodeVars[time_step][var_index] = expy.get_nodal_var(self.inFileId, time_step+1, var_index+1, self.numNodes)
        return self.nodeVars[time_step][var_index]
    

  def read(self, filename):

    self.inFileName = filename

    # open exodus database
    self.inFileId = expy.open(self.inFileName,self.iMode, \
                                          self.compWordSize,self.ioWordSize)
  
    # load database parameters
    self.title, self.numDimensions, self.numNodes, \
                 self.numElements, numBlocks, self.numNodeSets, \
                 self.numSideSets = expy.get_init(self.inFileId)

    # load connectivity
    self.coordinates = expy.get_coord(self.inFileId, \
                                                    self.numDimensions, \
                                                    self.numNodes)

    # load connectivity names
    self.coordinateNames = expy.get_coord_names(self.inFileId, \
                                                           self.numDimensions)

    # load block ids
    elementBlockIds = expy.get_elem_blk_ids(self.inFileId, \
                                                             numBlocks)

    # load blocks
    index = 0
    for blk_id in elementBlockIds:
      elem_type, num_elem, num_nodes_per_elem, num_attr = \
                               expy.get_elem_block(self.inFileId, blk_id)
      connect_numbered_from_one = expy.get_elem_conn(self.inFileId, \
                                                       blk_id, \
                                                       num_elem, \
                                                       num_nodes_per_elem)
      connect_numbered_from_zero = [ connect_numbered_from_one[i]-1 \
                                                       for i in range(len(connect_numbered_from_one)) ]
      self.elementBlocks.append(ElementBlock())
      self.elementBlocks[index].Initialize(blk_id,\
                                                         elem_type,\
                                                          self.numDimensions,\
                                                           num_elem,\
                                                           num_nodes_per_elem,\
                                                           num_attr,connect_numbered_from_zero, \
                                                         self.coordinates)
      index+=1

    # load node set ids
    if self.numNodeSets > 0:
      nodeSetIds = expy.get_node_set_ids(self.inFileId, self.numNodeSets);
      nodeSetNames = expy.get_names(self.inFileId, "ns", self.numNodeSets)
  
      # load node sets
      nsIndex = 0
      for ns_id in nodeSetIds:
        num_nodes_in_set, num_dist_in_set = expy.get_node_set_param(self.inFileId, ns_id)
        # ex_nodes numbers from 1
        ex_nodes = expy.get_node_set(self.inFileId, ns_id, num_nodes_in_set)
        # py_nodes numbers from 0
        py_nodes = [ node_id-1 for node_id in ex_nodes ]
        name = nodeSetNames[nsIndex]
        self.NodeSets.append(NodeSet(ns_id, num_nodes_in_set, num_dist_in_set, py_nodes, name))
        nsIndex += 1


    # load side set ids
    if self.numSideSets > 0:
      sideSetIds = expy.get_side_set_ids(self.inFileId, self.numSideSets);
      sideSetNames = expy.get_names(self.inFileId, "ss", self.numSideSets)

      # load side sets
      ssIndex = 0
      for ss_id in sideSetIds:
        num_sides_in_set, num_dist_in_set = expy.get_side_set_param(self.inFileId, ss_id)
        elems, sides = expy.get_side_set(self.inFileId, ss_id, num_sides_in_set)
        name = sideSetNames[ssIndex]
        self.SideSets.append(SideSet(ss_id, num_sides_in_set, num_dist_in_set, elems, sides, name))
        ssIndex += 1

    # read data
    self.numNodeVars = expy.get_var_param(self.inFileId, "n")
    self.nodeVarNames = expy.get_var_names(self.inFileId, "n", self.numNodeVars)
    self.numElemVars = expy.get_var_param(self.inFileId, "e")
    self.elemVarNames = expy.get_var_names(self.inFileId, "e", self.numElemVars)
    self.numGlobVars = expy.get_var_param(self.inFileId, "g")
    self.globVarNames = expy.get_var_names(self.inFileId, "g", self.numGlobVars)

    self.varTimes = expy.get_all_times(self.inFileId);

    self.nodeMap = expy.get_node_num_map(self.inFileId, self.numNodes)
    self.elemMap = expy.get_elem_num_map(self.inFileId, self.numElements)

    for itime in range(len(self.varTimes)):
      self.nodeVars.append([])
      for ivar in range(self.numNodeVars):
        self.nodeVars[itime].append( expy.get_nodal_var(self.inFileId, itime+1, ivar+1, self.numNodes) )
      self.elemVars.append([])
      for ivar in range(self.numElemVars):
        self.elemVars[itime].append([])
        for block in self.elementBlocks:
          self.elemVars[itime][ivar].append( \
               expy.get_elem_var(self.inFileId, itime+1, ivar+1, block.Id, block.numElements ))

    # close input database
    expy.close(self.inFileId)


  def write(self, filename):

    # self.

    self.outFileName = filename

    # open exodus database
    self.outFileId = expy.create(self.outFileName, self.comp_ws, self.io_ws)

    # initialize new database
    expy.put_init(self.outFileId, self.title, self.numDimensions,\
                             self.numNodes, self.numElements, len(self.elementBlocks),\
                             self.numNodeSets, self.numSideSets)

    # write coordinates
    expy.put_coord(self.outFileId, self.numDimensions, self.coordinates)

    # write coordinate names
    expy.put_coord_names(self.outFileId, self.coordinateNames)

    # write blocks
    for block in self.elementBlocks:
      expy.put_elem_block(self.outFileId, block.Id,\
                                   block.elemType, block.numElements,\
                                   block.numNodesPerElement, block.numAttr)
      connect_numbered_from_one = [block.connectivity[i]+1 for i in range(len(block.connectivity)) ]
      expy.put_elem_conn(self.outFileId, block.Id, connect_numbered_from_one)
    blockNames = [(block.name if block.name != "" else "block_"+str(block.Id)) for block in self.elementBlocks]
    expy.put_names(self.outFileId, "e", blockNames)

    # write sidesets
    sideSetNames = []
    for sideset in self.SideSets:
      expy.put_side_set_param(self.outFileId, sideset.Id, sideset.numSides, sideset.numDists)
      expy.put_side_set(self.outFileId, sideset.Id, sideset.elems, sideset.sides)
      sideSetNames.append(sideset.name)
    expy.put_names(self.outFileId, "ss", sideSetNames)

    # write nodesets
    nodeSetNames = []
    for nodeset in self.NodeSets:
      # convert nodes to count from 1
      ex_nodes = [node+1 for node in nodeset.nodes]
      expy.put_node_set_param(self.outFileId, nodeset.Id, nodeset.numNodes, nodeset.numDists)
      expy.put_node_set(self.outFileId, nodeset.Id, ex_nodes)
      nodeSetNames.append(nodeset.name)
    expy.put_names(self.outFileId, "ns", nodeSetNames)
  
    # write time steps
    if len(self.varTimes) > 0:
      time_step_id = 1
      for time_value in self.varTimes:
        expy.put_time(self.outFileId,time_step_id,time_value)
        time_step_id += 1
      
    # write variables
    if self.numElemVars > 0:
      expy.put_var_param(self.outFileId,"e",self.numElemVars)
      expy.put_var_names(self.outFileId,"e",self.numElemVars,self.elemVarNames)
      time_step_id = 1
      for timedump in self.elemVars:
        elemvar_index = 1 # exodus arrays count from 1
        for elemvar in timedump:
          block_index = 0 # python arrays count from 0
          for block in self.elementBlocks:
            expy.put_elem_var(self.outFileId,time_step_id,elemvar_index,block.Id, \
                              block.numElements, elemvar[block_index])
            block_index += 1
          elemvar_index += 1
        time_step_id += 1

    # write variables
    if self.numNodeVars > 0:
      expy.put_var_param(self.outFileId,"n",self.numNodeVars)
      expy.put_var_names(self.outFileId,"n",self.numNodeVars,self.nodeVarNames)
      time_step_id = 1
      for timedump in self.nodeVars:
        nodevar_index = 1 # exodus arrays count from 1
        for nodevar in timedump:
          expy.put_nodal_var(self.outFileId,time_step_id,nodevar_index, \
                            self.numNodes, nodevar)
          nodevar_index += 1
        time_step_id += 1

      
    # write element attributes
    if self.elemAttr:
      block_index = 0
      for block in self.elementBlocks:
        if block.numAttr:
          attr = self.elemAttr[block_index]
          expy.put_elem_attr(self.outFileId, block.Id, block.numAttr, block.numElements, attr)
        block_index += 1

    # close output database
    expy.close(self.outFileId)
