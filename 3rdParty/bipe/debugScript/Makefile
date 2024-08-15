all:
	make -C cnfuzzdd2009
	cp cnfuzzdd2009/cnfuzz .
	make clean -C cnfuzzdd2009
	make -C minisatSolver/core MROOT=..
	cp minisatSolver/core/minisat .
	make clean -C minisatSolver/core MROOT=..

clean:
	make clean -C cnfuzzdd2009 
	make clean -C minisatSolver/core MROOT=..
	rm -f cnfuzz
	rm -f minisat
	rm -f *~
