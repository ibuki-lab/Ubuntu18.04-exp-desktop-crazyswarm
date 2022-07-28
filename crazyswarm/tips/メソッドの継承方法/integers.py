#!/usr/bin/env python
#coding: UTF-8
import numpy as np
class integers(object):

    def __init__(self, x, y):
        
        self.x = x
        self.y = y

        self.z = 10
        self.hako = []

        self.a = np.array([[0, 0, 0, 0]])
    def main(self):


        for i in range(10):
            self.a[0, :] = np.array([1, 1, 1, 1])
            self.hako.append(self.a)
            print(id(self.hako[-1]))
        self.a[0, :] = np.array([2, 2, 2, 2])
        self.hako.append(self.a[0, :])
        print(self.hako)
integers(3, 4).main()


