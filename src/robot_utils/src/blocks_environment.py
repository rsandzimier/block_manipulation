#!/usr/bin/python
'''
# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)
'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import copy

class Planner(object):
    def __init__(self, start=None, goal=None, args=None):
        self.num_blocks =9

        self.bounds = ((-0.15,-0.15),(0.25,0.25))

        self.goal = {}
        n_rows = np.ceil(np.sqrt(self.num_blocks))
        spacing = 0.06
        offset = 0.03
        for i in range(self.num_blocks):
            self.goal[i] = self.Block(i,offset+spacing*(i%n_rows),offset+spacing*np.floor(i/n_rows),0.0,True)

        self.colors = ['white','yellow','blue','red','purple','orange','green','brown','black']

        start_prm = dict(self.goal)
        # start_prm[2] = copy.deepcopy(self.goal[2])
        # start_prm[2].x = 0.1811
        # start_prm[2].y = 0.1911
        # # start_prm[2].theta = 0.3
        # start_prm = self.sample_state(seed_state=dict(start_prm),sample_ids=[8])        
        start_prm = self.sample_state()

        # for i in range(self.num_blocks):
        #     if i%2==1:
        #         start_prm[i].in_place = True
            # if i!=2 and i!=8:
            #     start_prm[i].in_place = True

        goal_prm = copy.deepcopy(self.goal)

        for k in start_prm:
            print(start_prm[k].in_place)
        print('#######################')
        for k in goal_prm:
            print(goal_prm[k].in_place)
        # print(self.goal[8].x)
        # print(self.goal[2].x)


        # goal_prm[8].in_place = False
        # goal_prm[6].in_place = False

        self.plrs = self.plRS(start_prm, goal_prm, self.PRM, self.sample_state, self.connect_states)

        # Find shortest path to get list of actions and list of blocks that are in the way

        # actions, collisions, states, cost = self.prm.findShortestPath()

    class Block(object):
        def __init__(self, block_id, x, y, theta, in_place=False):
            self.block_id = block_id
            self.x = x
            self.y = y
            self.theta = theta
            self.in_place = in_place # In final location.
            self.width = 0.0508
        def __eq__(self, other): 
            if not isinstance(other, Planner.Block):
                return NotImplemented
            return self.block_id == other.block_id and self.x == other.x and self.y == other.y and self.theta == other.theta \
                and self.in_place == other.in_place and self.width == other.width
        def __ne__(self, other): 
            return not self == other

    class PickPlaceAction(object):
        def __init__(self, start, end, grasp_offset_90):
            '''
            start: Block object representing block starting position
            end: Block object representing block end position
            grasp_offset_90: Bool. False if grasp block faces at 0 and pi. True if grasp block faces at pi/2 and -pi/2
            '''
            self.start = start
            self.end = end
            self.grasp_offset_90 = grasp_offset_90

    class PushAction(object):
        def __init__(self, start, direction, distance):
            '''
            start: Block object representing block starting position
            direction: float. direction (rad) to push block (must be start.theta + k*pi/2. k=0,1,2,3)
            distance: float. distance (m) to push block

            '''
            self.start = start
            self.direction = direction
            self.distance = distance

    class plRS:
        def __init__(self,config_current, config_goal, motion_planner,state_sampler,state_connector):
            #add any hyper parameters here
            self.motion_planner =  motion_planner
            self.state_sampler = state_sampler
            self.state_connector = state_connector

            self.config_current = copy.deepcopy(config_current)
            self.config_goal = copy.deepcopy(config_goal)

            # print(blocks_remaining)
            # self.startplRS(blocks_remaining, config_current, config_goal)


        def startplRS(self):

            blocks_remaining = []
            for block_id in self.config_current:
                if not self.config_current[block_id].in_place:
                    blocks_remaining = blocks_remaining + [block_id]
            
            # blocks_remaining = [x for x in blocks_remaining if x != 8]
            print('#############STARTING##################')
            # display(self.config_current)
            # display(self.config_goal)

            actions, states = self.findplRSplan(0, blocks_remaining,self.config_current, self.config_goal)
            states = [copy.deepcopy(self.config_current)] + states
            return actions, states

        def findplRSplan(self, block_id, blocks_remaining, config_current, config_goal):
            # recursive function for solving block arrangement problem
            print('#############  NEW plRS recursion  ##################')
            # print("blocks remaining", blocks_remaining)
            # print("block to move",block_id)
            # display(config_current)
            # display(config_goal)

            config_goal_prm = copy.deepcopy(config_current)
            config_goal_prm[block_id] = copy.deepcopy(config_goal[block_id])
            path_U, blocks_in_way, states_U, cost = self.motion_planner(copy.deepcopy(config_current), config_goal_prm, block_id, self.state_sampler, self.state_connector, num_samples = 20).findShortestPath()
            states_U = states_U[1:]
            # for i,state in enumerate(states_U):
            #     # Planner.display(state)
            #     display(state)


            blocks_in_way_ids = [k for k in blocks_in_way]
            
            print("blocks remaining", blocks_remaining)
            # print("block to move",block_id)
            # print("PRM path_U", path_U)
            # print(path_U)
            # display(config_current)
            # display(config_goal_prm)
            # for i,state in enumerate(states_U):
            #     display(state)
                        
            if not blocks_in_way_ids:

                config_current[block_id] = copy.deepcopy(config_goal[block_id])
                blocks_remaining = [x for x in blocks_remaining if x != block_id]

                # config_current[block_id].in_place = True
                # print('no blocks in way')
                # print('blocks remaining', blocks_remaining)
                
                if not blocks_remaining:
                    # print('no blocks remaining so returning')
                    return copy.deepcopy(path_U), copy.deepcopy(states_U)

                for block_id_r in blocks_remaining:
                    blocks_remaining_without_r = [x for x in blocks_remaining if x != block_id_r]
                    # print('blocks remaining without r', blocks_remaining_without_r)
                    # print('block_id_r',block_id_r)
                    path, states = self.findplRSplan(block_id_r, blocks_remaining_without_r, config_current, config_goal)

                    if len(path)>0:
                        return copy.deepcopy(path_U) + copy.deepcopy(path), copy.deepcopy(states_U) + copy.deepcopy(states)
            else:

                block_id_b = blocks_in_way_ids[0]
                # print("block_b", block_id_b)
                # print("blocks_remaining", blocks_remaining)

                if block_id_b in blocks_remaining:

                    blocks_remaining_without_b = [x for x in blocks_remaining if x != block_id_b]
                    path_C, states_C, config_current = self.plRSclear(block_id_b,blocks_remaining_without_b,config_current,path_U)


                    if len(path_C)>0:

                        path, states  = self.findplRSplan(block_id, blocks_remaining, config_current, config_goal)
                     
                        if len(path)>0:
                            return copy.deepcopy(path_C) + copy.deepcopy(path), copy.deepcopy(states_C) + copy.deepcopy(states)

            return [], config_current


        def plRSclear(self,block_id, blocks_remaining, config_current, path_B):
            # clear path
            print('############CLEAR START################')

            intermediate_state= self.state_sampler(seed_state=copy.deepcopy(config_current), sample_ids=[block_id], in_place_only=False, epsilon=0.0)
            # print('CLEAR block_id',block_id)
            # print("blocks remaining", blocks_remaining)
            # display(config_current)
            # display(intermediate_state)
            # print('#############CLEAR END################')
            path_M, blocks_in_way, states_M, _  = self.motion_planner(copy.deepcopy(config_current), copy.deepcopy(intermediate_state), block_id, self.state_sampler, self.state_connector, num_samples = 20).findShortestPath()
            states_M =states_M[1:]
            # for i,state in enumerate(states_M):
            #     display(state)

            # config_current[block_id] = copy.deepcopy(intermediate_state[block_id])
            return path_M, states_M, intermediate_state

            # if not blocks_in_way:
            #     config_current[block_id] = intermediate_state[block_id]
            #     return config_current, path_U
            # else:
            #     block_id_b = blocks_in_way[0]
            #     if block_id_b in blocks_remaining:
            #         blocks_remaining_without_b = [x for x in blocks_remaining if x != block_id_b]
            #         config_current, path_C = plRSclear(block_id_b,blocks_remaining_without_b,config_current,[path_U,path_B])
            #         if len(path_C)>0:
            #             config_current, path = plRSclear(block_id_b,blocks_remaining,config_current,[path_B])
            #             if len(path)>0:
            #                 return config_current, path_C+path 


    class PRM(object):
        def __init__(self, start, goal, block_id, state_sampler, state_connector, num_samples = 100):
            '''
            start: dict of blocks representing starting configuration
            goal: dict of blocks representing goal configuration
            block_id: ID of block to place in goal position during this planning iteration
            state_sampler: function that can be called to sample a state
            num_samples: number of samples to add to PRM
            '''

            # Need to add some way of goal being "move block out of way randomly"
            self.nodes = {}
            self.block_id = block_id
            self.state_connector = state_connector
            self.addNode(self.Node(start,0))
            self.addNode(self.Node(goal,1))

            for s in range(num_samples):
                rand_state = state_sampler(seed_state=dict(start),sample_ids=[block_id],in_place_only=True, epsilon=0.2)
                self.addNode(self.Node(rand_state,s+2))

        class Node(object):
            def __init__(self, state, node_id):
                '''
                state: dict of blocks representing configuration at node
                node_id: int. unique id for each node. 0 for start. 1 for goal. any unique int for all other nodes
                connections: list of connections to other nodes. Note that connections are uni-directional
                '''
                self.node_id = node_id
                self.state = state 
                self.connections = []

                self.cost_from_start = np.inf
                if self.node_id == 0:
                    self.cost_from_start = 0
                self.via_connection = None

            def addConnection(self, connection):
                self.connections.append(connection)

        class Connection(object):
            def __init__(self,node_from,node_to,action,collisions=[],cost=0):
                self.node_from = node_from
                self.node_to = node_to
                self.action = action
                self.collisions = collisions
                self.cost = cost
                # Later on may need to add something like "traps" that keeps track of any blocks that get "trapped" (i.e. can't be moved)

        def addNode(self, node):
            for node_i in self.nodes.values():
                self.addConnections(node, node_i)
                self.addConnections(node_i, node)
            self.nodes[node.node_id] = node

        def addConnections(self, node_from, node_to):
            actions, collisions, costs = self.state_connector(node_from.state, node_to.state)
            for a,c,j in zip(actions, collisions, costs):
                node_from.addConnection(self.Connection(node_from,node_to, a, c, j))

        def findShortestPath(self):
            # Find shortest path from start to goal using PRM 

            # return:
            # actions: list of actions to get from start to goal. Can be PickPlaceAction object or PushAction object
            # collisions: dict of blocks that are in the way of this path (must be moved first)
            # states: list of states including the start and end state as well as any intermediate states
            # cost: float. total cost of path

            nodes = dict(self.nodes)

            while len(nodes) > 0:
                best_cost = np.inf
                best_node = None
                for node_id in nodes:
                    if nodes[node_id].cost_from_start <= best_cost:
                        best_cost = nodes[node_id].cost_from_start
                        best_node = nodes[node_id]
                if best_node.node_id == 1:
                    break
                for conn in best_node.connections:
                    if nodes.get(conn.node_to.node_id) is not None and best_cost + conn.cost < conn.node_to.cost_from_start:
                        conn.node_to.cost_from_start = best_cost + conn.cost
                        conn.node_to.via_connection = conn
                nodes.pop(best_node.node_id)


            node = self.nodes[1] # Goal node
            actions = []
            collisions = {}
            states = [node.state]
            cost = node.cost_from_start
            while node.via_connection is not None:
                conn = node.via_connection
                actions.append(conn.action)
                collisions.update(conn.collisions)
                node = conn.node_from
                states.append(node.state)

            actions = actions[::-1]
            states = states[::-1]

            return actions, collisions, states, cost

        # Start and goal
        # Nodes: State. connections
        # Connections: to, action, collisions, traps

    # Action
    # PickPlace. - Grasp orientation. Start. End. 
    # Push. Orientation. Start. 

    def blocks_collide(self,blockA,blockB,in_place_only=False, buffersA = None, buffersB = None):
        if buffersA is None:
            buffersA = (0.0,0.0,0.0,0.0)
        if buffersB is None:
            buffersB = (0.0,0.0,0.0,0.0) 
        buffer_ind = [[0,1],[2,1],[2,3],[0,3]] # Buffer indices (x,y) to use for each corner 
        signs = [[1,1],[-1,1],[-1,-1],[1,-1]] # Signs to use for each corner position

        half_width = blockA.width/2

        if in_place_only and not (blockA.in_place or blockB.in_place):
            return False

        for i in range(2):
            projection_angle = i*np.pi/2
            projected_corners_A = np.cos(blockA.theta+projection_angle)*blockA.x + \
                                  np.sin(blockA.theta+projection_angle)*blockA.y + np.array([-(half_width+buffersA[i+2]),half_width+buffersA[i]])
            no_collision = True
            below = False
            above = False
            for j in range(4):
                corner_x = (half_width + buffersB[buffer_ind[j][0]])*signs[j][0]
                corner_y = (half_width + buffersB[buffer_ind[j][1]])*signs[j][1]
                projected_corner_B = np.cos(blockA.theta+projection_angle)*(blockB.x + corner_x*np.cos(blockB.theta) - corner_y*np.sin(blockB.theta))+ \
                                     np.sin(blockA.theta+projection_angle)*(blockB.y + corner_x*np.sin(blockB.theta) + corner_y*np.cos(blockB.theta))
                if projected_corner_B >= projected_corners_A[0] and projected_corner_B <= projected_corners_A[1]:
                    no_collision = False
                    break
                elif projected_corner_B < projected_corners_A[0]:
                    below = True
                elif projected_corner_B > projected_corners_A[1]:
                    above = True
                if below and above:
                    no_collision = False
                    break                    
            if no_collision:
                return False

        for i in range(2):
            projection_angle = i*np.pi/2
            projected_corners_B = np.cos(blockB.theta+projection_angle)*blockB.x + \
                                  np.sin(blockB.theta+projection_angle)*blockB.y + np.array([-(half_width+buffersB[i+2]),half_width+buffersB[i]])
            no_collision = True
            below = False
            above = False
            for j in range(4):
                corner_x = (half_width + buffersA[buffer_ind[j][0]])*signs[j][0]
                corner_y = (half_width + buffersA[buffer_ind[j][1]])*signs[j][1]
                projected_corner_A = np.cos(blockB.theta+projection_angle)*(blockA.x + corner_x*np.cos(blockA.theta) - corner_y*np.sin(blockA.theta))+ \
                                     np.sin(blockB.theta+projection_angle)*(blockA.y + corner_x*np.sin(blockA.theta) + corner_y*np.cos(blockA.theta))
                if projected_corner_A >= projected_corners_B[0] and projected_corner_A <= projected_corners_B[1]:
                    no_collision = False
                    break
                elif projected_corner_A < projected_corners_B[0]:
                    below = True
                elif projected_corner_A > projected_corners_B[1]:
                    above = True
                if below and above:
                    no_collision = False
                    break          
            if no_collision:
                return False

        return True

    def block_collides_with_any(self, blocks, test_block,in_place_only=False, buffers_test_block=None, buffers=None):
        '''
        buffers_test_block: tuple or list of length 4 with buffer for each edge of test block
        buffers: dict of tuples/lists. Each is length 4. If this is not None, every block needs to be provided a buffer
        '''
        n_blocks = len(blocks)
        for key in blocks:
            buffersA = None if buffers is None else buffers[key]
            if self.blocks_collide(blocks[key],test_block, in_place_only=in_place_only, buffersA=buffersA, buffersB=buffers_test_block):
                return True

        return False 

    def block_collides_with_any_return_collisions(self, blocks, test_block, in_place_only=False, buffers_test_block=None):
        n_blocks = len(blocks)
        collisions = {}
        if in_place_only:
            soft_collisions = {}
        for key in blocks:
            is_collision = self.blocks_collide(blocks[key],test_block,buffersB=buffers_test_block)
            if in_place_only:
                is_soft_collision = is_collision
                is_collision = self.blocks_collide(blocks[key],test_block,in_place_only,buffersB=buffers_test_block)
                is_soft_collision = is_soft_collision and not is_collision
                if is_soft_collision:
                    soft_collisions[key] = blocks[key]
            if is_collision:
                collisions[key] = blocks[key]   

        if in_place_only:
            return collisions, soft_collisions
        return collisions  

    def block_in_bounds(self, block):

        return block.x >= self.bounds[0][0] and block.x <= self.bounds[1][0] and block.y >= self.bounds[0][1] and block.y <= self.bounds[1][1] 

    def block_pickable(self, state, block_id, grasp_offset_90, in_place_only=False):
        gripper_buffer = 0.05
        test_block = state[block_id]
        blocks = {s.block_id:s for s in state.values() if s.block_id!=block_id}
        if grasp_offset_90:
            buffers = (0.0,gripper_buffer,0.0,gripper_buffer)
        else:
            buffers = (gripper_buffer,0.0,gripper_buffer,0.0)
        collisions = self.block_collides_with_any_return_collisions(blocks, test_block, in_place_only=in_place_only,buffers_test_block=buffers)
        if in_place_only:
            soft_collisions = collisions[1]
            collisions = collisions[0]
            return len(collisions) == 0, soft_collisions
        return len(collisions) == 0

    def block_pushable(self, state, block_id, direction_int, distance, in_place_only=False):
        '''
        direction_int: int between 0 and 3 representing direction of push in block frame 0 (0.0), 1 (pi/2), 2 (pi), 3 (3pi/2)
        '''
        gripper_buffer = 0.05
        test_block = state[block_id]
        blocks = {s.block_id:s for s in state.values() if s.block_id!=block_id}

        buffers = [0.0,0.0,0.0,0.0]
        buffers[(direction_int+2)%4] = gripper_buffer
        buffers[direction_int] = distance

        collisions = self.block_collides_with_any_return_collisions(blocks, test_block, in_place_only=in_place_only,buffers_test_block=buffers)
        if in_place_only:
            soft_collisions = collisions[1]
            collisions = collisions[0]
            return len(collisions) == 0, soft_collisions
        return len(collisions) == 0

    def sample_state(self, seed_state=None, sample_ids=[], in_place_only=False, epsilon=0.0, clear_path_blocks_list=None, clear_path_buffers_list=None):
        '''
        seed_state (optional): dict. Any blocks that are not sampled copy the blocks from this dict. Must also provide sample_ids
        sample_ids (optional): list of ints. A list specifying which block ids to sample. Any block ids not included in this list
            should be copied from seed_state
        in_place_only (optional): Bool. If True, ignore collisions between blocks with in_place set to False. 
        epsilon (optional): float. Bias parameter. Probability that block is sampled along a perpendicular line from the block 
        clear_path_blocks_list (optional): list of blocks representing a path
        clear_path_buffers_list (optional): list of buffers for path 
        '''
        if seed_state is None:
            block_dict = {i:None for i in range(self.num_blocks)}
            sample_ids = range(self.num_blocks)
        else:
            block_dict = {s.block_id: None if s.block_id in sample_ids else s for s in seed_state.values()}

        if clear_path_blocks_list is None:
            clear_path_blocks_list = []

        if clear_path_buffers_list is None:
            clear_path_buffers_list = []

        clear_path_blocks = {}
        clear_path_buffers = {}
        for i in range(len(clear_path_blocks_list)):
            key = chr(ord('a')+i)# Use letter so that it is unique and not confused with numbered block_ids. Cannot use block_ids because the ids in 
                                #clear_path_blocks_list are not necessarily unique 
            clear_path_blocks[key] = clear_path_blocks_list[i]
            clear_path_buffers[key] = clear_path_buffers_list[i]

        num_blocks = len(block_dict)

        max_fails = 20*len(sample_ids)
        while True:
            blocks = dict(block_dict)
            fail_count = 0
            for i in sample_ids:
                while True:
                    new_block = self.sample_block(i,epsilon=epsilon)
                    blocks_no_none = {b.block_id:b for b in blocks.values() if b is not None} 
                    if not self.block_collides_with_any(blocks_no_none, new_block,in_place_only=in_place_only) and self.block_in_bounds(new_block) and \
                            not self.block_collides_with_any(clear_path_blocks, new_block, buffers=clear_path_buffers):
                        blocks[i]=new_block
                        break
                    fail_count += 1
                    if fail_count > max_fails:
                        break
                if blocks[i] is None:
                    break
            if sum([1 for b in blocks.values() if b is None]) == 0:
                return blocks

    def sample_block(self,block_id,epsilon=0.0):
        x_rand = np.random.uniform(self.bounds[0][0],self.bounds[1][0])
        y_rand = np.random.uniform(self.bounds[0][1],self.bounds[1][1])
        theta_rand = np.random.uniform(0,2*np.pi)
        if np.random.random() < epsilon:
            goal_block = [self.goal[i] for i in range(len(self.goal)) if self.goal[i].block_id == block_id][0]
            direction = goal_block.theta + np.random.randint(4)*np.pi/2
            distance = np.random.random()*2*goal_block.width
            x_rand = goal_block.x + distance*np.cos(direction)
            y_rand = goal_block.y + distance*np.sin(direction)
            theta_rand = goal_block.theta
        return self.Block(block_id, x_rand, y_rand, theta_rand)

    def connect_states(self,state_from,state_to,in_place_only=False):
        '''
        Searches for direct actions that can transition from state_from to state_to
        in_place_only - Bool. If True, only disallow collisions with blocks that are already in place.

        returns:
            actions - list of actions that will transition from state_from to state_to. empty list if none
            collisions - list of dicts of blocks. Elements of list correspond to the list of actions. The dict of blocks 
                contains all blocks that collide during the action. If in_place_only is set to False, the dicts will always be empty
            cost - list of floats representing costs for each action
        '''
        actions = []
        collisions = []
        costs = []
        if len(state_from) != len(state_to):
            return actions, collisions, costs # Different number of blocks in states. Undefined
        block_id = None
        for sf in state_from.values():
            st = state_to.get(sf.block_id)
            if st is None or sf.block_id != st.block_id:
                return actions, collisions, costs # Block ids don't match
            if sf != st:
                if block_id is None:
                    block_id = sf.block_id
                else:
                    return actions, collisions, costs # More than 1 block does not match. Can't connect using only 1 action
        if block_id is None:
            return actions, collisions, costs # States are exactly the same

        # Check if pick and place possible with grasp0.
        pickable, collisions_pick = self.block_pickable(state_from, block_id, False, True)
        placeable, collisions_place = self.block_pickable(state_to, block_id, False, True)
        if pickable and placeable:
            action = self.PickPlaceAction(state_from[block_id],state_to[block_id],False)
            collision = {}
            collision.update(collisions_pick)
            collision.update(collisions_place)
            cost = 1 + len(collision)
            actions.append(action)
            collisions.append(collision)
            costs.append(cost)

        # Check if pick and place possible with grasp90.
        pickable, collisions_pick = self.block_pickable(state_from, block_id, True, True)
        placeable, collisions_place = self.block_pickable(state_to, block_id, True, True)
        if pickable and placeable:
            action = self.PickPlaceAction(state_from[block_id],state_to[block_id],True)
            collision = {}
            collision.update(collisions_pick)
            collision.update(collisions_place)
            cost = 1 + len(collision)
            actions.append(action)
            collisions.append(collision)
            costs.append(cost)

        # Check if pushable
        sf = state_from[block_id]
        st = state_to[block_id]
        dx = st.x - sf.x
        dy = st.y - sf.y
        alpha = (np.arctan2(dy,dx)+2*np.pi)%(2*np.pi) # Between 0 and 2pi
        if sf.theta == st.theta and np.abs(sf.theta-alpha)%(np.pi/2) < 1e-5:
            direction_int = np.rint((sf.theta - alpha + 2*np.pi)%(2*np.pi)/(np.pi/2)).astype(int)
            distance = np.sqrt(dx**2+dy**2)
            pushable, collisions_push = self.block_pushable(state_from, block_id, direction_int, distance, True)
            if pushable:
                action = self.PushAction(state_from[block_id], alpha, distance)
                collision = {}
                collision.update(collisions_push)
                cost = 5 + len(collision)
                actions.append(action)
                collisions.append(collision)
                costs.append(cost)

        return actions, collisions, costs

    def display(self, blocks):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        r = patches.Rectangle((self.bounds[0][0],self.bounds[0][1]), self.bounds[1][0]-self.bounds[0][0], self.bounds[1][1]-self.bounds[0][1],fill=False)
        ax.add_patch(r)
        for blk in self.goal.values():
            r = patches.Rectangle((0,0), blk.width, blk.width,facecolor=self.colors[blk.block_id%9],edgecolor='k',alpha=0.1,hatch='x',linestyle='--')
            width = blk.width
            R = width*np.sqrt(2)/2
            x = blk.x
            y = blk.y
            theta = blk.theta
            t = mpl.transforms.Affine2D().rotate(theta) +  mpl.transforms.Affine2D().translate(x,y) - mpl.transforms.Affine2D().translate(R*np.cos(theta+np.pi/4),R*np.sin(theta+np.pi/4)) + ax.transData  #+  mpl.transforms.Affine2D().translate(x-R*np.cos(theta+np.pi/4)+width/2,y-R*np.sin(theta+np.pi/4)+width/2) + ax.transData 
            r.set_transform(t)
            ax.add_patch(r)
            ax.text(x, y, blk.block_id, rotation=np.degrees(theta),ha='center')

        for blk in blocks.values():
            r = patches.Rectangle((0,0), blk.width, blk.width,facecolor=self.colors[blk.block_id%9],edgecolor='k',alpha=0.4)
            width = blk.width
            R = width*np.sqrt(2)/2
            x = blk.x
            y = blk.y
            theta = blk.theta
            t = mpl.transforms.Affine2D().rotate(theta) +  mpl.transforms.Affine2D().translate(x,y) - mpl.transforms.Affine2D().translate(R*np.cos(theta+np.pi/4),R*np.sin(theta+np.pi/4)) + ax.transData  #+  mpl.transforms.Affine2D().translate(x-R*np.cos(theta+np.pi/4)+width/2,y-R*np.sin(theta+np.pi/4)+width/2) + ax.transData 
            r.set_transform(t)
            ax.add_patch(r)
            ax.text(x, y, blk.block_id, rotation=np.degrees(theta),ha='center')

        plt.xlim(self.bounds[0][0]-width, self.bounds[1][0]+width)
        plt.ylim(self.bounds[0][1]-width, self.bounds[1][1]+width)
        ax.axis('equal')
        plt.show()

def display(blocks):
    bounds = ((-0.15,-0.15),(0.25,0.25))
    colors = ['white','yellow','blue','red','purple','orange','green','brown','black']

    fig = plt.figure()
    ax = fig.add_subplot(111)
    r = patches.Rectangle((bounds[0][0],bounds[0][1]), bounds[1][0]-bounds[0][0], bounds[1][1]-bounds[0][1],fill=False)
    ax.add_patch(r)
    # for blk in goal.values():
    #     r = patches.Rectangle((0,0), blk.width, blk.width,facecolor=colors[blk.block_id%9],edgecolor='k',alpha=0.1,hatch='x',linestyle='--')
    #     width = blk.width
    #     R = width*np.sqrt(2)/2
    #     x = blk.x
    #     y = blk.y
    #     theta = blk.theta
    #     t = mpl.transforms.Affine2D().rotate(theta) +  mpl.transforms.Affine2D().translate(x,y) - mpl.transforms.Affine2D().translate(R*np.cos(theta+np.pi/4),R*np.sin(theta+np.pi/4)) + ax.transData  #+  mpl.transforms.Affine2D().translate(x-R*np.cos(theta+np.pi/4)+width/2,y-R*np.sin(theta+np.pi/4)+width/2) + ax.transData 
    #     r.set_transform(t)
    #     ax.add_patch(r)
    #     ax.text(x, y, blk.block_id, rotation=np.degrees(theta),ha='center')

    for blk in blocks.values():
        r = patches.Rectangle((0,0), blk.width, blk.width,facecolor=colors[blk.block_id%9],edgecolor='k',alpha=0.4)
        width = blk.width
        R = width*np.sqrt(2)/2
        x = blk.x
        y = blk.y
        theta = blk.theta
        t = mpl.transforms.Affine2D().rotate(theta) +  mpl.transforms.Affine2D().translate(x,y) - mpl.transforms.Affine2D().translate(R*np.cos(theta+np.pi/4),R*np.sin(theta+np.pi/4)) + ax.transData  #+  mpl.transforms.Affine2D().translate(x-R*np.cos(theta+np.pi/4)+width/2,y-R*np.sin(theta+np.pi/4)+width/2) + ax.transData 
        r.set_transform(t)
        ax.add_patch(r)
        ax.text(x, y, blk.block_id, rotation=np.degrees(theta),ha='center')

    plt.xlim(bounds[0][0]-width, bounds[1][0]+width)
    plt.ylim(bounds[0][1]-width, bounds[1][1]+width)
    ax.axis('equal')
    plt.show()

if __name__ == "__main__":
    P = Planner()
    actions,states = P.plrs.startplRS()

    for i,a in enumerate(actions):
        if type(a) == Planner.PickPlaceAction:
            print ("Pick",a.start.block_id,"at", (a.start.x,a.start.y,a.start.theta),"and place at",(a.end.x,a.end.y,a.end.theta))
        elif type(a) == Planner.PushAction:
            print ("Push",a.start.block_id,"at",(a.start.x,a.start.y,a.start.theta),"in direction",a.direction,"for distance",a.distance)
    for state in states:
        display(state)
