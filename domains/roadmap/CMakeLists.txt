add_executable (roadmap
  getopt1.c
  getopt.c
  graph_t.cc
  roadmap_t.cc
  roadmap.cc
  annealing.cc)

add_executable (gen-roadmap
  getopt1.c
  getopt.c
  graph_t.cc
  roadmap_t.cc
  annealing.cc)

add_subdirectory(benchmark)