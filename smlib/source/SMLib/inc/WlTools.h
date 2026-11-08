// SPDX-FileCopyrightText: Copyright (c) 2026 Adam Underkofler and WASMLib contributors (https://github.com/au12/WASMLib)
// SPDX-License-Identifier: Apache-2.0

// WASMLib: small helpers shared by WASMLib's speed changes to SMLib (listed in smlib/README.md).

#ifndef __WL_TOOLS_H__
#define __WL_TOOLS_H__

#include <SmTArray.h>

#include <algorithm>
#include <unordered_set>
#include <vector>

/*******************************************************************//**
WASMLib: keep the first of each repeated item, in order - what adding each item with AddUnique gives. AddUnique
  searches the whole array for every item, the square of the array's length, and topology lists (a face's edges,
  vertices or edgeuses) are gathered thousands of times in a boolean, some of them a hundred items long. Such lists
  seldom repeat an item, so a sorted copy usually shows there is nothing to remove.
***********************************************************************/
template <class TYPE>
inline void wl_KeepFirstOfEach(SmTArray<TYPE*> & rItems)
{
  const ULONG lSize = rItems.GetSize() ;
  if(lSize < 2) { return ; }
  if(lSize <= 16)
    {
      // short lists: the same search AddUnique made, in place
      ULONG lKept = 0 ;
      for(ULONG ii = 0; ii < lSize; ii++)
        {
          ULONG jj = 0 ;
          while(jj < lKept && rItems[jj] != rItems[ii]) { jj++ ; }
          if(jj == lKept) { rItems[lKept++] = rItems[ii] ; }
        }
      if(lKept < lSize) { rItems.SetSize(lKept) ; }
      return ;
    }
  std::vector<TYPE*> sSorted(lSize) ;
  for(ULONG ii = 0; ii < lSize; ii++) { sSorted[ii] = rItems[ii] ; }
  std::sort(sSorted.begin(), sSorted.end()) ;
  if(std::adjacent_find(sSorted.begin(), sSorted.end()) == sSorted.end()) { return ; }
  std::unordered_set<TYPE*> sSeen ;
  std::vector<TYPE*>        sKept ;
  for(ULONG ii = 0; ii < lSize; ii++) { if(sSeen.insert(rItems[ii]).second) { sKept.push_back(rItems[ii]) ; } }
  rItems.ReSet() ;
  for(TYPE *pItem : sKept) { rItems.Add(pItem) ; }
}

#endif // __WL_TOOLS_H__
