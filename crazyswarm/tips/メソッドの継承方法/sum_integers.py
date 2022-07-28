#!/usr/bin/env python
# coding: UTF-8

import integers as integer

class sum_or_diff_integers(integer.integers):

    def __init__(self, what, x, y):
        super(sum_or_diff_integers, self).__init__(x, y)

        self.what = what
        if self.what == '-':
            print(self.x - self.y)
        
        elif self.what == '+':
            print(self.x + self.y)
        
        else:
            print("what the fuck!!")
            print(self.z)

if __name__ == "__main__":
    sum_or_diff_integers(what=input(), x=2, y=1)
