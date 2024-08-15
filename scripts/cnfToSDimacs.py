#!/usr/bin/python3

import re
import os
import sys


assert(len(sys.argv) == 2)
pathBench = sys.argv[1]
print("c", pathBench)

exist1 = []
exist2 = []
random = []
clauses = []

nbVar = -1
cpt = 0
previous = '\0'

mapWeight = {}

# get the information.
with open(pathBench) as f:
    lines = f.readlines()

    for line in lines:
        if re.search("^p ", line):
            print(line, end='')
            nbVar = int(line.split(' ')[2])
        elif re.search("^c max ", line):
            for elt in line.split(' ')[2:-1]:
                exist1.append(int(elt))
        elif re.search("^c ind ", line):
            for elt in line.split(' ')[2:-1]:
                random.append(int(elt))
        elif re.search("^c p weight ", line):
            var = int(line.split(' ')[3])
            w = float(line.split(' ')[4])

            mapWeight[int(var)] = w
        else:
            clauses.append(line)


# print out the input following the sdimacs format.
bVar = [False for i in range(nbVar + 1)]
for e in exist1:
    bVar[e] = True
for r in random:
    bVar[r] = True
for i in range(1, nbVar):
    if not bVar[i]:
        exist2.append(i)

print('e', end=' ')
for e in exist1:
    print(e, end=' ')
print("0")

for r in random:
    assert r in mapWeight
    assert -r in mapWeight    
    w = round(mapWeight[r] / (mapWeight[r] + mapWeight[-r]),13)
    if w == 0:
        w = 0.00001
    print('r', '{:.13f}'.format(w), r, '0')

print('e', end=' ')
for e in exist2:
    print(e, end=' ')
print("0")


for cl in clauses:
    print(cl, end='')
