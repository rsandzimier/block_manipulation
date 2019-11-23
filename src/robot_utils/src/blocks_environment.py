#!/usr/bin/python
'''
# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)
'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

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

        [self.Block(0,0.03,0.03,0.0),self.Block(1,0.09,0.03,0.0),self.Block(2,0.03,0.09,0.0),self.Block(3,0.09,0.09,0.0)]
        #self.goal = [self.Block(0,0.03,0.03,0.0),self.Block(1,0.09,0.03,0.0),self.Block(2,0.03,0.09,0.0),self.Block(3,0.09,0.09,0.0)]

        self.colors = ['white','yellow','blue','red','purple','orange','green','brown','black']

    class Block:
        def __init__(self, block_id, x, y, theta, in_place=False):
            self.block_id = block_id
            self.x = x
            self.y = y
            self.theta = theta
            self.in_place = in_place # In final location.
            self.width = 0.0508

    def valid_state(self,blocks):
        n_blocks = len(blocks)
        for i in range(n_blocks):
            # Check that each block is in bounds
            for j in range(i+1,n_blocks):
                if blocks_collide(blocks[i],blocks[j]):
                    return False
        return True

    def blocks_collide(self,blockA,blockB):
        R = blockA.width*np.sqrt(2)/2
        half_width = blockA.width/2

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

    def block_collides_with_any(self, blocks, test_block):
        n_blocks = len(blocks)
        for i in range(n_blocks):
            if self.blocks_collide(blocks[i],test_block):
                return False
        return True 

    def sample_state(self):
        max_fails = 20*self.num_blocks
        while True:
            blocks = []
            fail_count = 0
            for i in range(self.num_blocks):
                while True:
                    new_block = self.sample_block(i)
                    if self.block_collides_with_any(blocks, new_block):
                        blocks.append(new_block)
                        break
                    fail_count += 1
                    if fail_count > max_fails:
                        break
                if len(blocks) != i+1:
                    break
            if len(blocks) == self.num_blocks:
                return blocks

    def sample_block(self,block_id):
        x_rand = np.random.uniform(self.bounds[0][0],self.bounds[1][0])
        y_rand = np.random.uniform(self.bounds[0][1],self.bounds[1][1])
        theta_rand = np.random.uniform(0,2*np.pi)
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

        plt.show()


P = Planner()
for i in range(15):
    S = P.sample_state() 
    P.display(S)




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
