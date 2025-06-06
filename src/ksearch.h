// -*- coding: utf-8 -*-
// ksearch.h
// -----------------------------------------------------------------------------
//
// Started on <lun 07-08-2023 13:53:29.565134107 (1691409209)>
// Carlos Linares López <carlos.linares@uc3m.es>
// Ian Herman <iankherman@gmail.com>   Ian Herman <iankherman@gmail.com>

//
// Main include file
//

#ifndef _KSEARCH_H_
#define _KSEARCH_H_

// definitions
#define CMAKE_BUILD_TYPE "Release"
#define CMAKE_VERSION "1.6"

// ***initialization ***
#include "KHSdefs.h"

// *** data structures

// --- node
#include "structs/KHSnode_t.h"
#include "structs/KHSbacknode_t.h"
#include "structs/KHSlabelednode_t.h"

// --- open list
#include "structs/KHSbucket_t.h"

// --- closed list
#include "structs/KHSclosed_t.h"

// -- solutions
#include "structs/KHSsolution_t.h"
#include "structs/KHSksolution_t.h"
#include "structs/KHSksolutions_t.h"

// *** search algorithms

// --- base definition
#include "algorithm/KHSbsolver.h"

// m-Dijkstra
#include "algorithm/KHSmA.h"

// Bidirectional Edge-Labeled A* (BELA*)
#include "algorithm/KHSbela.h"

// K* (Ian's Implementation)
#include "algorithm/KHSIK.h"

// K* (Ian's Implementation, Heuristic enabled)
#include "algorithm/KHSIKStar.h"

#endif // _KSEARCH_H_

// Local Variables:
// mode:cpp
// fill-column:80
// End:
