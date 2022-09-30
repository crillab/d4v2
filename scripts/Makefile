all:
	make -C cnfuzzdd2009
	cp cnfuzzdd2009/cnfuzz .
	make clean -C cnfuzzdd2009
	make -C minisatSolver/core
	cp minisatSolver/core/minisat .
	make clean -C minisatSolver/core

clean:
	make clean -C cnfuzzdd2009 
	make clean -C minisatSolver/core
	rm -f minisatSolver/mtl/template.mk
	rm -f cnfuzz
	rm -f minisat
	rm -f *~
