# Introduction #

This repository presents a solution for the DYMACS dataset. The problem of this 
dataset is related with non-admissible heuristics when using Greatest Circle 
Distance due to errors when collecting the information.

For solving this problem, a solution based on a Simmulated Annealing (SA) is 
proposed.

# Install #

To download the code type the following:

``` sh
    $ git clone https://github.com/clinaresl/ksearch.git
```

To compile the source code, create first the `Makefile` with:

```sh
mkdir build && cd build
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DDATA_NY=ON \
  -D7ZIP_LOCATION=$(which 7z) \
  ..
```

```sh
cd .. 
make roadmap
```

# Execution #

Execution example with the NY dataset.

```sh
./domains/roadmap/roadmap \
  --graph domains/roadmap/benchmark/USA-road-d.NY.gr
```

# Debug #

Using lldb, you can execute:

```bash
lldb domains/roadmap/roadmap
target create ./domains/roadmap/roadmap
process launch -- --graph domains/roadmap/benchmark/USA-road-d.NY.gr
```

# Dataset #

The roadmap domain is taken from the [9th DIMACS Implementation Challenge:
Shortest Paths](http://www.diag.uniroma1.it/~challenge9/download.shtml#benchmark)

The available benchmarks are stored in the directory `benchmarks`. The graphs
are suffixed with `.gr` where, every line consists of an edge identified by its
two vertices represented with integer indices, and the third value is the edge
cost. Data lines are preceded with `a` whereas comments are preceded with the
hash symbol `#`. See the available
[help](http://www.diag.uniroma1.it/~challenge9/format.shtml) for more
information on the formats.

# License #

MIT License

Copyright (c) 2025, Javier Martín

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


# Authors #

Carlos Linares Lopez <carlos.linares@uc3m.es>  
Computer Science and Engineering Department <https://www.inf.uc3m.es/en>  
Universidad Carlos III de Madrid <https://www.uc3m.es/home>

Javier Martín Pizarro <jmartinpizarro04@gmail.com>  
Computer Science and Engineering Department <https://www.inf.uc3m.es/en>  
Universidad Carlos III de Madrid <https://www.uc3m.es/home>

