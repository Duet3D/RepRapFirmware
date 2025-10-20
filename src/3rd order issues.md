# 3rd order motion code issues

- [done] MovementProfile::NonDecelDistance returns incorrect value when T4 segment has been combined with T2
- [done] PlanMoves doesn't maintain constant extrusion speed. When the moves are extruding, PlanMoves should construct the movement plan in terms of extrusion distance.
  [done anoither way] Perhaps we should normalise the moves so that totalDistance is the extrusion distance, not the axis dstance?
- [done] PlanMoves allows speed to be increased above requestedSpeed because it takes maximum requestedSpeed of the set of moves
- We don't yet handle plans with no constant speed segment and either a constant acceleration or constant deceleration segment but not both
- Need multi-phase plans for sequences of moves for which the extrusion rate changes by more than a small amount. If there is a large reduction in extrusion rate, then plan from that change to the end of the move, to see whether we can decelerate to zero if we start at that extrusion rate. If not then find the highest steady extrusion rate that we can decelerate from. Then plan the first part to end at that extrusion rate. Alternative: plan the whole sequence at the lower (second) extrusion rate. If the resulting plan has a constant speed segment that continues up to the boundary between first and second segments, try to re-plan the first segment.