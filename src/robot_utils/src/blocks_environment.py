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

class Planner:
    def __init__(self, start=None, goal=None, args=None):
        self.num_blocks = 4
        self.bounds = ((0.0,0.0),(0.25,0.25))

        self.goal = []
        n_rows = np.ceil(np.sqrt(self.num_blocks))
        spacing = 0.06
        offset = 0.03
        for i in range(self.num_blocks):
            self.goal.append(self.Block(i,offset+spacing*(i%n_rows),offset+spacing*np.floor(i/n_rows),0.0))

        self.colors = ['white','yellow','blue','red','purple','orange','green','brown','black']

        # Create PRM to find path to get block 0 to goal position
        self.prm = self.PRM(self.sample_state(), self.goal, 0, self.sample_state, num_samples = 100)
        # Find shortest path to get list of actions and list of blocks that are in the way
        actions, blocks_in_way = self.prm.findShortestPath()
        print( actions, blocks_in_way)


    class Block:
        def __init__(self, block_id, x, y, theta, in_place=False):
            self.block_id = block_id
            self.x = x
            self.y = y
            self.theta = theta
            self.in_place = in_place # In final location.
            self.width = 0.0508

    class PickPlaceAction:
        def __init__(self, start, end, grasp_offset_90):
            '''
            start: Block object representing block starting position
            end: Block object representing block end position
            grasp_offset_90: Bool. False if grasp block faces at 0 and pi. True if grasp block faces at pi/2 and -pi/2
            '''
            self.start = start
            self.end = end
            self.grasp_offset_90 = grasp_offset_90

    class PushAction:
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
        def __init__(self, motion_planner):
            #add any hyper parameters here
            self.motion_planner = motion_planner
            pass

        def findplRSplan(block_id, blocks_remaining, config_current, config_goal):
            # recursive function for solving block arrangement problem
            path_U = []
            path_M, blocks_in_way = self.motion_planner(start, goal, block_id, state_sampler)
            
            if not blocks_in_way:
                config_current[block_id] = copy.copy(config_goal[block_id])

                if not blocks_remaining:
                    return path_U

                for block_id_r in blocks_remaining:
                    blocks_remaining_without_r = [x for x in blocks_remaining if x != block_id_r]
                    path = pathfindplRSplan(block_id_r, blocks_remaining_without_r, config_current, config_goal)
                    if len(path)>0:
                        return path_U + path
            else:
                block_id_b = blocks_in_way[0]:
                if block_id_b in blocks_remaining:
                    config_current, path_C = plRSclear(block_id,blocks_remaining_without_b,config_current,config_goal):
                    if len(path_C)>0:
                        path = pathfindplRSplan(block_id, blocks_remaining, config_current, config_goal)
                        if len(path)>0:
                            return path_C+path


        def plRSclear(block_id,blocks_remaining,config_current,path_B):
            # clear path
            intermediate_state = sample_state(seed_state=None, sample_ids=[block_id],in_place_only=False, epsilon=0.0)
            path_U, blocks_in_way = self.motion_planner(config_current, intermediate_state, block_id, state_sampler)
            if not blocks_in_way:
                config_current[block_id] = intermediate_state[block_id]
                return config_current, path_U
            else:
                block_id_b = blocks_in_way[0]:
                if block_id_b in blocks_remaining:
                    blocks_remaining_without_b = [x for x in blocks_remaining if x != block_id_b]
                    config_current, path_C = plRSclear(block_id_b,blocks_remaining_without_b,config_current,[path_U,path_B])
                    if len(path_C)>0:
                        config_current, path = plRSclear(block_id_b,blocks_remaining,config_current,[path_B])
                        if len(path)>0:
                            return config_current, path_C+path 

        
            






                






    class PRM:
        def __init__(self, start, goal, block_id, state_sampler, num_samples = 100):
            '''
            start: list of blocks representing starting configuration
            goal: list of blocks representing goal configuration
            block_id: ID of block to place in goal position during this planning iteration
            state_sampler: function that can be called to sample a state
            num_samples: number of samples to add to PRM
            '''

            # Need to add some way of goal being "move block out of way randomly"
            self.nodes = []
            self.block_id = block_id
            self.addNode(self.Node(start,0))
            self.addNode(self.Node(goal,1))

            for s in range(num_samples):
                rand_state = state_sampler(seed_state=start[:],sample_ids=[block_id],in_place_only=True, epsilon=0.2)
                self.addNode(self.Node(rand_state,s+2))

        class Node:
            def __init__(self, state, node_id):
                '''
                state: list of blocks representing configuration at node
                node_id: int. unique id for each node. 0 for start. 1 for goal. any unique int for all other nodes
                connections: list of connections to other nodes. Note that connections are uni-directional
                '''
                self.node_id = node_id
                self.state = state
                self.connections = []

        class Connection:
            def __init__(self,to,action,collisions=[],traps=[]):
                self.to = to
                self.action = action
                self.collisions = collisions
                self.traps = traps
                self.cost = 0.0 # Cost based on action (pushing costs more) and number of collisions, etc

        def addNode(self, node):
            # Append to self.nodes
            # Check all connections and add any as necessary
            for i in range(len(self.nodes)):
                # Add connections if there is an action that can connect each pair of nodes
                pass
            self.nodes.append(node)

        def findShortestPath(self):
            # Find shortest path from start to goal using PRM 

            # return:
            # actions: list of actions to get from start to goal. Can be PickPlaceAction object or PushAction object
            # in way: list of blocks that are in the way of this path (must be moved first)
            
            # Not implemented
            # Use below for hard-coded testing
            return [Planner.PickPlaceAction(Planner.Block(0,0.0,0.0,0.0),Planner.Block(0,0.0,0.0,0.0),False),Planner.PushAction(Planner.Block(0,0.0,0.0,0.0),0.0,0.0)], [Planner.Block(1,0.0,0.0,0.0),Planner.Block(2,0.0,0.0,0.0)]


        # Start and goal
        # Nodes: State. connections
        # Connections: to, action, collisions, traps

    # Action
    # PickPlace. - Grasp orientation. Start. End. 
    # Push. Orientation. Start. 

    def valid_state(self,blocks):
        n_blocks = len(blocks)
        for i in range(n_blocks):
            # Check that each block is in bounds
            for j in range(i+1,n_blocks):
                if blocks_collide(blocks[i],blocks[j]):
                    return False
        return True

    def blocks_collide(self,blockA,blockB,in_place_only=False):
        R = blockA.width*np.sqrt(2)/2
        half_width = blockA.width/2

        if in_place_only and not (blockA.in_place or blockB.in_place):
            return False

        for projection_angle in np.linspace(0,np.pi/2,2):
            projected_corners_A = np.cos(blockA.theta+projection_angle)*blockA.x + \
                                  np.sin(blockA.theta+projection_angle)*blockA.y + np.array([-half_width,half_width])
            no_collision = True
            for corner_angle in np.linspace(0,2*np.pi,4,False):
                projected_corner_B = np.cos(blockA.theta+projection_angle)*(blockB.x + R*np.cos(blockB.theta+corner_angle+np.pi/4))+ \
                                     np.sin(blockA.theta+projection_angle)*(blockB.y + R*np.sin(blockB.theta+corner_angle+np.pi/4))
                if projected_corner_B >= projected_corners_A[0] and projected_corner_B <= projected_corners_A[1]:
                    no_collision = False
                    break
            if no_collision:
                return False
        for projection_angle in np.linspace(0,np.pi/2,2):
            projected_corners_B = np.cos(blockB.theta+projection_angle)*blockB.x + \
                                  np.sin(blockB.theta+projection_angle)*blockB.y + np.array([-half_width,half_width])
            no_collision = True
            for corner_angle in np.linspace(0,2*np.pi,4,False):
                projected_corner_A = np.cos(blockB.theta+projection_angle)*(blockA.x + R*np.cos(blockA.theta+corner_angle+np.pi/4))+ \
                                     np.sin(blockB.theta+projection_angle)*(blockA.y + R*np.sin(blockA.theta+corner_angle+np.pi/4))
                if projected_corner_A >= projected_corners_B[0] and projected_corner_A <= projected_corners_B[1]:
                    no_collision = False
                    break
            if no_collision:
                return False
        return True

    def block_collides_with_any(self, blocks, test_block,in_place_only=False):
        n_blocks = len(blocks)
        for i in range(n_blocks):
            if self.blocks_collide(blocks[i],test_block, in_place_only=in_place_only):
                return False
        return True 

    def block_in_bounds(self, block):
        return block.x >= self.bounds[0][0] and block.x <= self.bounds[1][0] and block.y >= self.bounds[0][1] and block.y <= self.bounds[1][1] 

    def sample_state(self, seed_state=None, sample_ids=[],in_place_only=False, epsilon=0.0):
        '''
        seed_state (optional): list. Any blocks that are not sampled copy the blocks from this list. Must also provide sample_ids
        sample_ids (optional): list of ints. A list specifying which block ids to sample. Any block ids not included in this list
            should be copied from seed_state
        in_place_only (optional): Bool. If True, ignore collisions between blocks with in_place set to False. 
        epsilon (optional): float. Bias parameter. Probability that block is sampled along a perpendicular line from the block 
        '''
        if seed_state is None:
            block_list = [None]*self.num_blocks
            sample_ids = range(self.num_blocks)
        else:
            block_list = [None if s.block_id in sample_ids else s for s in seed_state]

        num_blocks = len(block_list)

        max_fails = 20*len(sample_ids)
        while True:
            blocks = block_list
            fail_count = 0
            for i in sample_ids:
                while True:
                    new_block = self.sample_block(i,epsilon=epsilon)
                    blocks_no_none = [b for b in blocks if b is not None]
                    if self.block_collides_with_any(blocks_no_none, new_block,in_place_only=in_place_only) and self.block_in_bounds(new_block):
                        blocks[i]=new_block
                        break
                    fail_count += 1
                    if fail_count > max_fails:
                        break
                if blocks[i] is None:
                    break
            if blocks.count(None) == 0:
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


    def display(self, blocks):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        r = patches.Rectangle((self.bounds[0][0],self.bounds[0][1]), self.bounds[1][0]-self.bounds[0][0], self.bounds[1][1]-self.bounds[0][1],fill=False)
        ax.add_patch(r)
        for blk in self.goal:
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

        for blk in blocks:
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


P = Planner()
for i in range(15):
    S = P.sample_state() 
    P.display(S)

# for n in P.prm.nodes:
#     P.display(n.state)





        # Tree/graph of bunch of states

        # Collision detector (list of blocks) -> Bool

        # class Tree:
        #     self.root = Node

        # class Node:
        #     self.blocks = # List of blocks
        #     self.children = # List of nodes



        # Tree/graph Connections of states
        
        # Block class
        # State is a list of blocks
        

        # Boundaries
        # Goal configuration
        # Starting configuration

        # Methods pick/place and push

        # Pick/place: state, final_state -> 

        # Collision checker
        # State sampler

        # Tree - Many states
        # State - Many blocks
