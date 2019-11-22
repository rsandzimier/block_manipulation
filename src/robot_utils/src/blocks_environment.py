#!/usr/bin/python
'''
# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)
'''
import numpy as np

class Planner:
    def __init__(self, start, goal, args):
        pass

    class Block:
        def __init__(self, block_id, x, y, theta, in_place=False):
            self.block_id = block_id
            self.x = x
            self.y = y
            self.theta = theta
            self.in_place = in_place # In final location.
            self.width = 50.8


    def valid_state(self,blocks):
        n_blocks = len(blocks)
        for i in range(n_blocks):
            # Check that each block is in bounds
            for j in range(i+1,n_blocks):
                if blocks_collide(blocks[i],blocks[j]):
                    return False
        return True    

    def blocks_collide(blockA,blockB):
        blockA.theta
        R = blockA.width*np.sqrt(2)/2
        half_width = blockA.width/2

        for i in range(2):
            projected_corners_A = np.cos(blockA.theta+i*np.pi/2)*blockA.x + \
                                  np.sin(blockA.theta+i*np.pi/2)*blockA.y + np.array([-half_width,half_width])
            no_collision = True
            for j in range(4):
                projected_corner_B = np.cos(blockA.theta+i*np.pi/2)*(blockB.x + R*np.cos(blockB.theta+np.pi/4+j*np.pi/2))+ \
                                     np.sin(blockA.theta+i*np.pi/2)*(blockB.y + R*np.sin(blockB.theta+np.pi/4+j*np.pi/2))
                print "Test", projected_corners_A, projected_corner_B
                if projected_corner_B >= projected_corners_A[0] and projected_corner_B <= projected_corners_A[1]:
                    no_collision = False
                    print "collision"
                    break
            if no_collision:
                return False
        for i in range(2):
            projected_corners_B = np.cos(blockB.theta+i*np.pi/2)*blockB.x + \
                                  np.sin(blockB.theta+i*np.pi/2)*blockB.y + np.array([-half_width,half_width])
            no_collision = True
            for j in range(4):
                projected_corner_A = np.cos(blockB.theta+i*np.pi/2)*(blockA.x + R*np.cos(blockA.theta+np.pi/4+j*np.pi/2))+ \
                                     np.sin(blockB.theta+i*np.pi/2)*(blockA.y + R*np.sin(blockA.theta+np.pi/4+j*np.pi/2))
                print "Test", projected_corners_B, projected_corner_A
                if projected_corner_A >= projected_corners_B[0] and projected_corner_A <= projected_corners_B[1]:
                    no_collision = False
                    print "collision"
                    break
            if no_collision:
                return False
        return True

    def sample_state(self):
        pass

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
