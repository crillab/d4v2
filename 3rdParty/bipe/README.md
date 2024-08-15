**bipe** is a preprocessor that leverages definability, i.e., the 
ability to determine that some (complex) gates are implied by the input formula Σ. 
Such gates can be exploited to simplify Σ without modifying its number of models. 
Our preprocessing technique thus consists of two phases: computing a bipartition of 
the variables of Σ where the variables from O are defined in Σ in terms of I,
then eliminating some variables of O in Σ. **bipe** can also be used to only extract 
a set of input variables by using a dedicated option. It is also possible to specify 
which variables are part of the output/input set by spotting them in the input file
(see the section about spotting variables).

# How to use

## How to compile

To compile **bipe** please use the following command line:

```bash
./build.sh
```

You can compile **bipe** in static, in debug mode or as a library (see the section about 
how to use **bipe** as a library).

To compile in static mode, please use:

```bash
./build.sh -s
```

To compile in debug mode, please use:

```bash
./build.sh -d
```

To build the library, please use the following command line::

```bash
./build.sh -l
``` 

## Help me!

Please use the help (-h) to get information about the way of running **bipe**.

```
$ ./build/bipe -h
Some parameters are missing, please read the README
USAGE: ./build/bipe_debug -i CNF_INPUT -m METHOD
Options:
  -h [ --help ]                        Help screen
  -i [ --input ] arg                   (required) Path to get the input file.
  -m [ --method ] arg                  (required) The method we use 
                                       (bipartition - compute a bipartition of 
                                       input and output variables, backbone - 
                                       compute the backbone of the formula and 
                                       dac - compute a directed acyclic graph
                                       definability circuit (DAC) to get a bipartition).
  --solver-name arg (=Glucose_bipe)         The SAT solver used.
  --solver-limited arg (=0)            The number of allowed conflicts (0 
                                       stands for infinity).
  --bipartition-sorting arg (=OCC_ASC) The order used for selecting the 
                                       variable we first try to put in the 
                                       output set (OCC_ASC - w.r.t the 
                                       occurrence, RANDOM, NATURAL_ORDER or 
                                       GEN_TAUTS - w.r.t. the number of 
                                       tautological clauses generated).
  --bipartition-use-dac arg (=1)       Initializing the bipartition by computing a 
                                       directed acyclic definability circuit.
  --bipartition-use-backbone arg (=1)  Computing backbone to spot output 
                                       variables before computing the 
                                       bipartition.
  --dac-use-backbone arg (=1)          Computing backbone to spot output 
                                       variables before running DAC.
  --backbone-reverse-polarity arg (=1) When calling the solver for computing 
                                       the backbone we consider in priority the
                                       reverse polarity
  --timeout arg (=0)                   The timeout chosen in second(s).
```

## Input formula

**bipe** uses as input formula a CNF formula following the DIMACS format [1].
It is possible to force some variables to be in the input or the output set specifically.
To do it, you need to specify in the input file which variables are selected.
For instance, to specify input variables **{1,2}** you need to add the comment line **c p protected 1 2 0** in your input file.
To specify output variables **{2,3}** you need to add the comment line **c p show 2 3 0** in your input file.
Output variables can also be considered as the set of variables which is not projected. 
Consequently, forcing output variables is required when using **bipe** for projected model counting.
Here is an example:

```
$ cat forceOutput.cnf
p cnf 6 7
c p show 2 5 6 0
5 6 0
-1 -2 3 0
1 -3 0
2 -3 0
-1 -2 4 0
1 -4 0
2 -4 0

$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000112
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0


$ cat instanceTest/forceInput.cnf 
p cnf 6 7
c p protected 3 0
5 6 0
-1 -2 3 0
1 -3 0
2 -3 0
-1 -2 4 0
1 -4 0
2 -4 0

$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000214
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0
```

## Computing the backbone

The backbone of a formula is the set of literals which are true in every satisfying truth assignment.
Because these literals are units, they can be freely put in the set of output variables and then it makes sense to compute them first
when one searches in identifying a bipartition. **bipe** is able to extract the backbone of a CNF formula in the following way:

```
$ ./build/bipe -i instanceTest/testUnit.cnf -m backbone
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000127
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 3
c [PREPOC BACKBONE] Number of units detected by calling the solver: 1
v 2 3 4 
```

To compute the backbone, **bipe** calls a SAT solver in order to find the literals that are part of it.
To do so, **bipe** starts with a model (in the case the formula is SAT since in the case the formula is UNSAT the notion 
of backbone does not really make sense) and tests whether the formula conjointed with the negation of a literal taken from this model 
is SAT or not. If the resulting formula is UNSAT, then the literal belongs to the backbone. When the formula is SAT 
it is possible to spot literals that do not belong to the backbone by marking the literals that are assigned differently 
regarding the initial model. In order to enpower this approach, it is possible to activate an option 
(it is activated by default) that helps by assigning differently the literal by considering the negation 
of the saved polarity in the SAT solver (this option makes sense if the SAT solver is used in incremental mode and 
if it uses the polarity saving heuristic [2]). This option can be activated/deactivated by running one of the following command lines:

```
$ ./build/bipe -i instanceTest/testUnit.cnf -m backbone --backbone-reverse-polarity 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000198
c [PREPOC BACKBONE] Reverse polarity option: 0
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 3
c [PREPOC BACKBONE] Number of units detected by calling the solver: 1
v 2 3 4 

$ ./build/bipe -i instanceTest/testUnit.cnf -m backbone --backbone-reverse-polarity 1
c [PREPOC BACKBONE] Time to compute the backbone: 0.000198
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 3
c [PREPOC BACKBONE] Number of units detected by calling the solver: 1
v 2 3 4 
```

## Computing a Directed Acyclic Definability Circuit

In order to get a bipartition, it is also possible to consider the input/output variables of a directed acyclic definability circuit (DAC).
A DAC is a graph which has an acyclic (i.e., loop-free or feed-forward) topology. Constructing such a definability circuit
can be computationally expensive in the general case, but it is possible to leverage Boolean Constraint Propagation (BCP)
to detect gates efficiently (in polynomial time) or to detect gates syntactically [5]. To construct a bipartition 
via a DAC please run the following command line.

```
$ ./build/bipe -i instanceTest/cnf8.cnf -m dac
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000204
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 1
c [DAC] #Equivalences: 3
c [DAC] #And gates: 1
c [DAC] #XOR gates: 1
v 1 5 8 10 12 15 16 17 18 19 20 22 0
```

By default this method computes the backbone first. But it is possible to turn off this option as shown in the following
command line:

```
$ ./build/bipe -i instanceTest/cnf8.cnf -m dac --dac-use-backbone 0
c [DAC] Preprocessing backbone: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 1
c [DAC] #Equivalences: 3
c [DAC] #And gates: 1
c [DAC] #XOR gates: 1
v 1 5 8 10 12 15 16 17 18 19 20 22 0
```


## Computing a bipartition

The main objective of **bipe** is to extract a bipartition that separates the variable into two groups:
the input variables and the output variables (that are the variables that are defined logically w.r.t. the input set).
To do so, **bipe** uses the techniques presented in [3]. To run **bipe** please use the following command line:

```
$ ./build/bipe -i instanceTest/testUnit.cnf -m bipartition
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 5
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000224
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 3
c [PREPOC BACKBONE] Number of units detected by calling the solver: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 9
c [BIPARTITION] Initial input set: 2
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 2
c [BIPARTITION] Time needed to compute the partition: 0.00
v 1 5 0
```

In the following we present the heuristic that can be leveraged in order to speed up **bipe**.

### Ordering heuristic for constructing a bipartition

There exist 4 heuristics you can use for constructing a bipartition:

* NATURAL_ORDER, as the name suggests it, this order considers the variables in the natural order;
* RANDOM, as the name suggests it, the variables are considered randomly;
* OCC_ASC, this heuristic considers in priority the variables which maximize the number of occurrences
 in the initial formula;
* GEN_TAUTS, this heuristic considers in priority the variables which maximize the number of generated 
  tautological clauses.

Feel free to add your own heuristics!

### Extracting the backbone in preprocessing

As pointed out in the section presenting the backbone extraction process implemented in **bipe**, 
the backbone can always be viewed as a part of the output variables. To activate/deactivate this option 
(which is actually activated by default), please use one of the following command line:

```
$ ./build/bipe -i instanceTest/testUnit.cnf -m bipartition --bipartition-use-backbone 1
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 5
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000213
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 3
c [PREPOC BACKBONE] Backbone size: 3
c [PREPOC BACKBONE] Number of units detected by calling the solver: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 9
c [BIPARTITION] Initial input set: 2
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 2
c [BIPARTITION] Time needed to compute the partition: 0.00
v 1 5 0


$ ./build/bipe -i instanceTest/testUnit.cnf -m bipartition --bipartition-use-backbone 0
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 0
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 5
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 6
c [BIPARTITION] Initial input set: 2
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 2
c [BIPARTITION] Time needed to compute the partition: 0.00
v 1 5 0
```

### Using gates to spot some output variables in preprocessing

It is also possible to use gate detection in order to speed up the bipartition extraction.
This process is run before starting the bipartition extraction as a preprocessing step.
To consider this option, please use the following command line:

```
$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition --bipartition-use-dac 1
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.000777
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0
```

It is possible to combine the preprocessing options.

```
$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition --bipartition-use-dac 1 --bipartition-use-backbone 1
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.001124
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0
```

By default, *bipartition-use-backbone* and *bipartition-use-dac* are activated. 


# SAT solver

**bipe** is highly dependant to a SAT solver. In **bipe** we use the SAT solver **Glucose_bipe v.3** (by default) which 
has been designed to incremental SAT solving [4]. However, you can plug your own solver by implementing 
the c++ interface *WrapperSolver*. You can then run your solver by specifying its name on the command line in the
following way:

```
$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition --solver-name Glucose_bipe
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.001094
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0
```

In the case you have restricted resources in order to compute a bipartition or a backbone, you can specify 
a budget, in term of number of conflicts, the solver has (by default the number of conflicts is 0 which means that
there is no limit). If you want to give a budget of 500 conflicts you can use the following command line:

```
$ ./build/bipe -i instanceTest/forceOutput.cnf -m bipartition --solver-limited 500
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 500
c [BIPARTITION] #Projected: 3
c [BIPARTITION] #Protected: 0
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.001089
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 2
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 3
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.00|      0.00|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.00|      0.00|      0.00|      0.00|         3|         3|         0|         0 |
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 3
c [BIPARTITION] Time needed to compute the partition: 0.00
v 2 5 6 0
```

# Timeout

It is possible to set a budget for the method you want to run. This feature is quite important since 
the method we use for computing the *backbone* or a *bipartition* requires to call a SAT solver 
which can take forever. In order to get at least an approximation in the case when the formula is too 
hard so that computing exactly the *backbone* or a *bipartition* is out of reach, we implemented a mechanism that asynchronously 
stops the SAT solver, stops the extraction process in progress and then returns a partial solution
computed so far. To compute a *bipartition* with a budget of 1 second, you can use the following command line:

```
$ ./build/bipe -i instanceTest/hard.cnf -m bipartition --timeout 1
c [BIPARTITION] Method used for sorting: OCC_ASC
c [BIPARTITION] Preprocessing backbone: 1
c [BIPARTITION] Preprocessing DAC: 1
c [BIPARTITION] #Conflicts for the SAT solver: 0
c [BIPARTITION] #Projected: 390
c [BIPARTITION] #Protected: 78
c [DAC] Preprocessing backbone: 1
c [CONSTRUCTOR] Solver: Glucose_bipe
c [PREPOC BACKBONE] Time to compute the backbone: 0.154991
c [PREPOC BACKBONE] Reverse polarity option: 1
c [PREPOC BACKBONE] Number of SAT calls: 16
c [PREPOC BACKBONE] Backbone size: 0
c [PREPOC BACKBONE] Number of units detected by calling the solver: 0
c [CONSTRUCTOR] Solver: Glucose_bipe
c [DAC] #Equivalence classes: 0
c [DAC] #Equivalences: 0
c [DAC] #And gates: 0
c [DAC] #XOR gates: 0
c [BIPARTITION] #Gates: 0
c [BIPARTITION] Initial input set: 312
c [CONSTRUCTOR] Solver: Glucose_bipe
c [HEURISTIC-BIPARTITION] Constructor
c [HEURISTIC-BIPARTITION] Method run: OCC_ASC
c Bipartition in progress
c ------------------------------------------------------------------------------------------
c |      time| time(SAT)| time(UNS)| time(UND)|     #call|#call(SAT)|#call(UNS)|#call(UND) |
c ------------------------------------------------------------------------------------------
c |      0.01|      0.01|      0.00|      0.00|         1|         1|         0|         0 |
c |      0.03|      0.03|      0.00|      0.00|         2|         2|         0|         0 |
c |      0.03|      0.03|      0.00|      0.00|         3|         3|         0|         0 |
c |      0.05|      0.05|      0.00|      0.00|         4|         4|         0|         0 |
c |      0.05|      0.05|      0.00|      0.00|         5|         5|         0|         0 |
c |      0.05|      0.05|      0.00|      0.00|         6|         6|         0|         0 |
c |      0.07|      0.07|      0.00|      0.00|         7|         7|         0|         0 |
c |      0.08|      0.08|      0.00|      0.00|         8|         8|         0|         0 |
c |      0.09|      0.09|      0.00|      0.00|         9|         9|         0|         0 |
c |      0.09|      0.09|      0.00|      0.00|        10|        10|         0|         0 |
c |      0.09|      0.09|      0.00|      0.00|        11|        11|         0|         0 |
c |      0.13|      0.13|      0.00|      0.00|        12|        12|         0|         0 |
c |      0.13|      0.13|      0.00|      0.00|        13|        13|         0|         0 |
c |      0.19|      0.19|      0.00|      0.00|        14|        14|         0|         0 |
c |      0.19|      0.19|      0.00|      0.00|        15|        15|         0|         0 |
c |      0.21|      0.21|      0.00|      0.00|        16|        16|         0|         0 |
c |      0.29|      0.29|      0.00|      0.00|        17|        17|         0|         0 |
c |      0.30|      0.30|      0.00|      0.00|        18|        18|         0|         0 |
c |      0.30|      0.30|      0.00|      0.00|        19|        19|         0|         0 |
c |      0.30|      0.30|      0.00|      0.00|        20|        20|         0|         0 |
c |      0.37|      0.37|      0.00|      0.00|        21|        21|         0|         0 |
c |      0.38|      0.38|      0.00|      0.00|        22|        22|         0|         0 |
c |      0.39|      0.39|      0.00|      0.00|        23|        23|         0|         0 |
c |      0.42|      0.42|      0.00|      0.00|        24|        24|         0|         0 |
c |      0.42|      0.42|      0.00|      0.00|        25|        25|         0|         0 |
c |      0.45|      0.45|      0.00|      0.00|        26|        26|         0|         0 |
c |      0.45|      0.45|      0.00|      0.00|        27|        27|         0|         0 |
c |      0.47|      0.47|      0.00|      0.00|        28|        28|         0|         0 |
c |      0.50|      0.50|      0.00|      0.00|        29|        29|         0|         0 |
c |      0.59|      0.59|      0.00|      0.00|        30|        30|         0|         0 |
c |      0.61|      0.61|      0.00|      0.00|        31|        31|         0|         0 |
c |      0.61|      0.61|      0.00|      0.00|        32|        32|         0|         0 |
c |      0.72|      0.72|      0.00|      0.00|        33|        33|         0|         0 |
c |      0.72|      0.72|      0.00|      0.00|        34|        34|         0|         0 |
c |      0.72|      0.72|      0.00|      0.00|        35|        35|         0|         0 |
c |      0.73|      0.73|      0.00|      0.00|        36|        36|         0|         0 |
c |      0.74|      0.74|      0.00|      0.00|        37|        37|         0|         0 |
c |      0.74|      0.74|      0.00|      0.00|        38|        38|         0|         0 |
c |      0.74|      0.74|      0.00|      0.00|        39|        39|         0|         0 |
c |      0.75|      0.75|      0.00|      0.00|        40|        40|         0|         0 |
c |      0.75|      0.75|      0.00|      0.00|        41|        41|         0|         0 |
c |      0.78|      0.78|      0.00|      0.00|        42|        42|         0|         0 |
c |      0.78|      0.78|      0.00|      0.00|        43|        43|         0|         0 |
c |      0.78|      0.78|      0.00|      0.00|        44|        44|         0|         0 |
c |      0.78|      0.78|      0.00|      0.00|        45|        45|         0|         0 |
c |      0.81|      0.81|      0.00|      0.00|        46|        46|         0|         0 |
c [MAIN] Method stop
c [Glucose_bipe] Stop Glucose_bipe
c |      0.83|      0.81|      0.00|      0.01|        47|        46|         0|         1 |
c [BIPARTITION] Stop by signal, number of remaining undefined variables: 265
c [BIPARTITION] Statistics 
c [BIPARTITION] #Input variables computed: 390
c [BIPARTITION] Time needed to compute the partition: 0.83
v 2 3 4 6 7 8 9 10 11 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 35 36 37 38 41 42 44 45 46 47 48 49 50 51 52 53 55 56 57 59 61 62 64 65 66 67 69 70 71 72 74 75 76 77 78 81 82 83 84 85 86 87 88 89 90 91 92 93 94 95 96 97 98 99 100 101 103 104 105 106 108 109 110 111 113 115 116 117 119 122 123 126 128 129 130 131 133 134 135 137 138 139 141 143 144 145 146 148 149 150 151 152 153 155 156 157 158 159 160 161 162 163 164 165 166 167 168 170 171 172 173 174 175 176 177 178 180 182 183 184 185 187 188 189 190 193 194 195 196 197 198 199 200 202 203 204 205 206 207 208 210 211 212 213 214 215 216 218 219 220 221 222 223 225 226 227 228 229 230 231 232 234 235 236 237 238 239 240 241 242 244 245 246 247 248 249 251 252 253 254 255 256 257 259 260 261 262 263 264 266 267 268 269 270 271 272 273 274 276 277 278 280 281 282 283 284 285 286 287 289 291 292 293 294 295 296 297 298 299 300 301 302 303 304 306 308 309 311 312 313 314 315 319 320 322 323 324 326 327 328 330 331 332 333 334 335 336 337 338 339 340 341 342 343 344 348 349 351 352 353 355 356 357 358 359 361 362 365 368 369 373 375 376 379 382 383 384 385 386 387 388 389 390 393 394 396 398 399 400 401 402 403 404 405 406 407 409 410 413 414 415 416 418 419 420 422 423 425 427 428 429 430 432 433 434 436 437 440 441 442 443 444 445 446 447 448 449 450 452 453 454 455 456 457 458 459 460 461 462 463 464 465 466 468 469 470 471 472 473 475 476 477 478 479 480 481 482 483 484 485 486 487 488 0
```


# Implementing your ideas in **bipe**

**bipe** proposes a bunch of scripts that can help you during your development phase. 
You can try to detect if your approach is correct by generating benchmarks randomly:

```
$ cd debugScript
$ ./searchBadExitQuick.sh ./testBiPartitionWithChecker.sh
```

# How to use **bipe** as a library

**TODO!**


# References

[1] Satisfiability suggested format, 1993

[2] K. Pipatsrisawat and A. Darwiche. A lightweight component caching scheme for satisfiability solvers. In SAT, pages 294–299, 2007.

[3] Jean-Marie Lagniez, Emmanuel Lonca, Pierre Marquis: Definability for model counting. Artif. Intell. 281: 103229 (2020)

[4] Gilles Audemard, Jean-Marie Lagniez, Laurent Simon: Improving Glucose_bipe for Incremental SAT Solving with Assumptions: Application to MUS Extraction. SAT 2013: 309-317

[5] Jean-Marie Lagniez, Pierre Marquis: On Preprocessing Techniques and Their Impact on Propositional Model Counting. 
J. Autom. Reason. 58(4): 413-481 (2017)
