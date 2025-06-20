# Introduction #

This library implements a number of search algorithms to compute *K*
(non-simple) optimal paths. It provides both *brute-force* and *heuristic*
search algorithms. It also comes with a number of domain-dependent solvers that
test the performance of these algorithms

# Dependencies #

Tests have been created using the [Google Test
Framework](https://github.com/google/googletest) which is necessary for both
compiling and runing the tests, see below.

In addition, a number of scripts are provided for various purposes. They all
have been implemented in python 3.8.

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

# Tests #

This library comes with a number of unit tests that have been implemented with
[Google Test](https://github.com/google/googletest). If Google Test is available
in your computer, then you can compile and run the tests with:

``` sh
    $ make gtest
    $ tests/gtest
```

from the `ksearch/` directory created after the clone.

# Search algorithms #

The search algorithms provided in this library solve the *k* non-simple
shortest-paths problem. It provides both brute-force and heuristic search
algorithms:

* Brute-force search algorithms:

  + `mDijkstra`: It consists of expanding every node up to *k* times
  + `K0`: Uninformed variant of K*
  + `belA0`: it is the uninformed variant of BELA*

* Heuristic search algorithms:

  * `mA*`: Informed variant of mDijkstra
  + `K*`: Original algorithm used for solving the K shortest-path problem with heuristics
  + `belA*`: Algorithm used for solving the K shortest-path problem which is guaranteed to expand nodes only once

# Domain-dependent solvers #

This package comes with a number of domain-dependent solvers under `domains/`:

* *2D Pathfinding*
* *Grid*
* *N-Pancake*
* *N-Puzzle*
* *Roadmap* 

Every domain-dependent solver provides all the search algorithms described
above. For further details, check out the documentation under `domains/`

# Publications #

* Linares López, Carlos; Herman, Ian, 2024. Evolving A* to Efficiently Solve the
  k Shortest-Path Problem. Proceedings of the Twenty-Seventh European Conference
  on Artificial Intelligence (ECAI 2024), pages 4352-4359. Santiago de Compostela (Spain).
  
An extended version of the same paper with an example of BELA0 and the results
of all the empirical evaluation can be found at:

* Linares López, Carlos; Herman, Ian, 2024. Evolving A* to Efficiently Solve the
  k Shortest-Path Problem (Extended Version). arXiv 2024. eprint
  [2408.08227](https://arxiv.org/abs/2408.08227).

In addition, all the data used in the experimentation is available in Zenodo:

* Linares López, C., & Herman, I. (2024). Experimental evaluation of BELA0/BELA* (ECAI 2024) (Data set). Evolving A* to Efficiently Solve the k Shortest-Path Problem (ECAI), Santiago de Compostela (Spain). Zenodo. https://doi.org/10.5281/zenodo.13293103
  

# License #

MIT License

Copyright (c) 2016, 2023, Carlos Linares López, Ian Herman

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

Ian Herman <iankherman@gmail.com>  
Computer Science and Engineering Department https://www.inf.uc3m.es/en  
Universidad Carlos III de Madrid https://www.uc3m.es/home

