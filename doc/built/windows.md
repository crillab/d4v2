# d4

This is a built version of d4.
The project is licensed under the LGPL-2.1 and the source can be found at https://github.com/SoftVarE-Group/d4v2

All dependencies are in `bin`.

## Usage

The binary `d4.exe` is inside `bin`.
To show the help message, use:

```
d4.exe --help
```

### d-DNNF compilation

```
d4.exe --input /path/to/input.cnf --method ddnnf-compiler --dump-ddnnf /path/to/output.ddnnf
```

... will take a CNF from `/path/to/input.cnf`, compile it into a d-DNNF and write it to `/path/to/output.ddnnf`.
