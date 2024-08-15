#!/usr/bin/python3

from pysat.solvers import Solver
import sys
import re

assert len(sys.argv) == 3


def printProgressBar(i, max, postText):
    n_bar = 10  # size of progress bar
    j = i/max
    sys.stdout.write('\r')
    sys.stdout.write(
        f"[{'=' * int(n_bar * j):{n_bar}s}] {int(100 * j)}%  {postText}")
    sys.stdout.flush()


# get the solution
fileSolution = sys.argv[2]
fileCnf = sys.argv[1]

# get the solution
pattern = "v "
file = open(fileSolution, "r")
solution = []
for line in file:
    if re.search(pattern, line):
        solution = line.split()[1:]
        solution.pop()
        for i in range(len(solution)):
            solution[i] = int(solution[i])
file.close()

# get the formula information (clauses and projected)
file = open(fileCnf, "r")
cnf = []
projected = []
protected = []
nbVar = 0
hasProjected = False

for line in file:
    if re.search("c p show", line):
        hasProjected = True
        projected = line.split()[3:]
        projected.pop()
        for i in range(len(projected)):
            projected[i] = int(projected[i])
    if re.search("c p protected", line):
        protected = line.split()[3:]
        protected.pop()
        for i in range(len(protected)):
            protected[i] = int(protected[i])
    if re.search("p cnf ", line):
        nbVar = int(line.split()[2])
    elif re.search("c ", line):
        continue
    else:
        clause = line.split()[:-1]
        tmp = []
        for e in clause:
            tmp.append(int(e))
        cnf.append(tmp)
file.close()

# update the projected set regarding the input cnf.
if not hasProjected:
    projected = [i + 1 for i in range(nbVar)]
else:
    projected.sort()

# check that the solution is in the projected set
isInSolution = [False for i in range(nbVar+1)]
for e in solution:
    isInSolution[e] = True

isProjected = [False for i in range(nbVar+1)]
for e in projected:
    isProjected[e] = True

for i in range(1, nbVar+1):
    if isInSolution[i] and not isProjected[i]:
        print(i, "is in the solution but it is not a projected var.")
        sys.exit(1)  # error

# Fill the solver with problem.
s = Solver(name='Cadical153')

# Encode the selected equivalences
for i in range(1, nbVar + 1):
    cl = [-i, -(nbVar + i), (2*nbVar + i)]
    s.add_clause(cl)
    cl = [-i, (nbVar + i), -(2*nbVar + i)]
    s.add_clause(cl)

# duplicate the problem twice
for cl in cnf:
    if(len(cl) == 0):
        continue
    cl1 = []
    cl2 = []

    for e in cl:
        if e < 0:
            cl1.append(-nbVar + e)
            cl2.append(-(2*nbVar) + e)
        else:
            cl1.append(nbVar + e)
            cl2.append(2*nbVar + e)

    # add to the solver.
    s.add_clause(cl1)
    s.add_clause(cl2)

# check that all the protected variables are in the solution
for e in protected:
    if e not in solution:
        print(e, "is not in the solution found", solution)
        exit(1)  # error


# check the solution (that means we check that all the variables not in the
# solution are defined from the variables given in the solution, we do not
# check for minimality)
assums = solution

for i in range(1, nbVar+1):
    printProgressBar(i, nbVar, "check solution")
    if not isInSolution[i] and isProjected[i] and\
            s.solve(assumptions=assums + [i+nbVar, -i - (2*nbVar)]):
        print("")
        print(i, "is not defined from the given solution.")
        sys.exit(1)  # error, i is not defined.
print("")


# check for minimality.
for i in range(1, nbVar+1):
    printProgressBar(i, nbVar, "check minimality of the solution")

    if isInSolution[i] and isProjected[i] and\
            not s.solve(assumptions=list(set(assums) - {i}) + [i+nbVar, -i - (2*nbVar)]):
        print("")
        print(i, "can be defined from the other variables.")
        sys.exit(1)  # error, i is not defined.
print("")

sys.exit(0)
