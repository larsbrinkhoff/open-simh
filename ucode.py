#!/usr/bin/env python3

import re
import sys

address = None
microword = 0

def output():
    global address, microword
    if address == None:
        return
    x0 = (microword >> 60) & 0o7777777777
    x1 = (microword >> 30) & 0o7777777777
    x2 = (microword >>  0) & 0o7777777777
    print(f"ROM[{address}][0] = 0{x0:010o};")
    print(f"ROM[{address}][1] = 0{x1:010o};")
    print(f"ROM[{address}][2] = 0{x2:010o};")
    microword = 0

def process(line):
    global address, microword
    m = re.compile(r'^([0-7]*): ').match(line)
    if m:
        output()
        address = m.group(1)
    m = re.compile(r'^ *[.].* = [1-9][0-9]* *([0-9,]+)\r*$').match(line)
    if m:
        for bit in m.group(1).split(','):
            if bit:
                microword |= 0o400000000000000000000000000000 >> int(bit)

if __name__ == "__main__":
    for file in sys.argv:
        with open(file) as f:
            for line in f:
                process(line)
