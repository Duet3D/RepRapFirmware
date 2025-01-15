Multiple motion systems in RRF 3.6
==================================

Multiple motion systems present challenges in RRF 3.6 because the final movement is generated using a single set of DriveMovement objects;
whereas in 3.5 the DMs were attached to the DDAs, which were in separate queues for each MS. This is how we handle these channelges in 3.6.

The overall movement scheme is:
- There is one DDARing for each MS
- Each move generated in a particular MS is put into a DDA in the corresponding DDA ring
- When a move is committed, MoveSegments for the logical drives concerned are generated from the DDA. These segments are attached to the DMs in the common set
- After a DDA is committed, the DDA is used only for tracking when the move should complete, reporting on its parameters (acceleration, top speed etc.)
- The DDA no longer stores the Cartesian coordinates of the end of each move. Instead we store the cartesian coordinates of the end of each move in the DDARing.

Objectives
- A MS must 'own' a logical drive before it can move it. At most one MS can own a logical drive at any one time.
- When one MS releases a drive and another allocates it, the final drive endpoint set by the old owner must be copied to the new owner, and axis coordinates updated appropriately
- The fact that a drive endpoint has changed in the MS because of a change in ownership must not cause the motion system to attempt to move from the old endpoint to the new one

Recording of endpoints:
- Each DDARing maintains a copy of endpoints of the last move that was added to it. Only the endpoints of drives that the corresponding MS owns should be considered valid.
- We also maintain a global copy of the endpoints of drives that are not owned (lastKnownEndpoints). These are set during initialisation and updated whenever a MS releases a drive.
- Each DMs maintain its endpoint in 'currentMotorPosition'
- Each DDA has 'endpoint'. These are the planned endpoints for the move that the DDA represents. Homing and probing moves will generally complete before some of those endpoints are reached.

System invariants:
- When no moves are queued and any corrections for homing or calibration have been applied:
-- For drives that are owned, the values for those drives in the DDARing of the owning MS match the motor positions stored in the DMs
-- For drives that are not owned, the values for those drives in lastKnownEndpoints match the motor positions stored in the DMs

To handle allocation of axes and the associated drives:
- Before a MS can do a normal move, the axes mentioned in the corresponding command (or implied in the case of a probing move) must be allocated to that MS.
- Before a MS can do a raw motor move, the drives mentioned in the corresponding command must be allocated to that MS.
- To allocate an axis, we allocate drives that control that axis. We get that set of drives from the kinematics.
- To allocate a set of drives we first check that none of them is already owned by another MS. Then we allocate them to this MS; then copy the lastKnownEndpoints for those drives to the corresponding DDARing;
   then transform the whole set of DDA endpoints to the new machine coordinates of that MS; then transform them to the new user coordinates of that MS.
- Because allocating an axis can change the user position recorded in the MS, we must do the allocation right at the start of processing G0/1/2/3 commands, before we make any use of the initial position.
- To release a set of drives we first copy their last endpoints from the DDARing of the owning MS to lastKnownEndpoints; then we flag those drives as unowned.

Initialising a standard move in the DDA:
- We must start from the correct set of endpoints. If we have just allocated one or more drives then these may not be the same as the endpoints store in the previous DDA entry.
   This could be solved by storing in the DDARing a set of endpoints representing the start positions of the next move to be added to the ring.
   But that doesn't solve the next issue. So we update the endpoints in the previous move instead, This is safe as long as the ownedDrives bits for the updated endpoints are not set.
- We must start frm the correct set of coordinates so that we can set dv in the DDA correctly.

Preparing (committing) a move in the DDA:
- DDA::Prepare uses the difference in endpoints stored in the previous move and endpoint of the new move to get the number of steps required.
   This is a problem if the previous move had the wrong endpoints for these drives because they were not owned.
   The solution we adopt is: (1) in each DDA store the bitmap of owned drives. Don't move any other drives. (2) when the endpoints in a MS need to be updated, update them in the previous move, like we always used to do.

Special situations:
- At initialisation time: DONE
   We take the assumed initial machine position for the kinematics (GCodes::Reset)
   We transform that machine position to lastKnownEndpoints and also store them in the motor positions in the DMs (MovementState::GlobalInit, called for GCodes::Reset)
   We store the assumed initial machine position and the corresponding user position in all MSs (MovementState::Init)
   We also store the endpoints in the DDA rings (MovementState::Init)

- When steps/mm or microstepping is changed (M92 and M350): DONE
   We need to adjust all sets of endpoints (lastKnownEndpoints, the motor positions and the endpoints in the DDA ring) by the ratio of new to old steps/mm.
   This matters especially on a delta where the assumed initial position does not correspond to the endpoints all being zero.

- When we execute G92: DONE
   We allocate the axes (and extruders) that the G92 command refers to, and hence the drives affected (GCodes::SetPositions)
   We transform the G92 coordinates to machine coordinates and store them in the current MS (GCodes::SetPositions)
   We transform the machine coordinates into drive endpoints (GCodes::SetPositions)
   We store those endpoints in the DDARing for the MS concerned and in the motor positions in the DMs (MovementState::SetNewPositionOfOwnedAxes)

- When auto-calibrating a delta printer and adjusting positions to take account of endstop offsets: DONE
   We fetch the endpoints from the DDARing for the MS that did the calibration to lastKnownEndpoints (MovementState::AdjustMotorPositions)
   We adjust those endpoints as instructed by the calibration (MovementState::AdjustMotorPositions)
   We also store the new endpoints in our DDARing and in the motor positions in the DMs (MovementState::AdjustMotorPositions)

- When we complete a simulation and restore the user and machine positions: DONE
   Currently we save the user position of each MS in its restore point, then try to restore endpoints based on them.
   A problem with that is that any axes that are unowned wont get restored.
   Maybe it's better instead at the start of the simulation to have all MSs save the endpoints of their owned drives to lastKnownEndPoints and save their current tool numbers, release all drives, and then save lastKnownEndpoints.
   When simulation ends we can restore lastKnownEndpoints from the saved copy, set the DDARing last moves endpoints and the motor endpoints to them, restore the current tool in the MSs,
   and transform lastKnownEndpoints back to machine and user coordinates in each MS .
   
   Old idea was:
   In each movement state: (GCodes::EndSimulation)
     We restore the user position and the current tool to the values in the restore point in that MS
     We transform the restored position to machine coordinates and to endpoints
   We store those endpoints in lastKnownEndpoints from the restored position and copy them into the DDARings
   We also store them in the motor positions in the DMs in case the transformation left us with slightly different endpoints from previously

- When we do an endstop-sensitive move on a Cartesian or Core machine and that axis is controlled by drives that don't affect other axes:
   We only need to stop the axis whose endstop triggered, if other axes are being homes simultaneously
   We retrieve the new endpoint for those drives (usually only one) from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints
   If it was a homing move then the system will change the machine coordinate of that axis, transform it to endpoints, and update endpoints (similar to executnig a G92 command for that axis)

- When we do an endstop-sensitive move on an axis on a Core machine and that axis is controlled by drives that affect other axes:
   We stop all drives
   We retrieve the new endpoints for all drives owned by the MS that initiated the homing move from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints

- When we do an endstop-sensitive move on a machine that homes using raw motor moves:
   We stop the drive whose endstop triggered 
   We copy its endpoint from the DM motor position into the DDARing of the MS that initiated the move and into lastKnownEndpoints

- When we do a probing move and the probe triggers:
   We stop all drives
   We retrieve the new endpoints for all drives owned by the MS that initiated the homing move from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints

- When we apply babystepping:
   Currently babystepping assumes that Z motion and not other motion is controlled only by the Z drive. It adjusts the Z position and the endpoint of the Z driver in existing moves in the queue.
   We must also update the Z drive endpont in the DDARing by the amount of Z babystepping pushed throuh the queue.

- When we pause a print and some moves in the queue are thrown away:
   We must set the endpoint in the DDA ring in which moves have been thrown away to the endpont of the last completed move

- When we do an emergency pause and throw away all moves in the queue:
   If we are definitely going to stop, we needn't update endpoints. Otherwise, we must set the endpoint in the DDA ring in which moves have been thrown away to the endpont of the last completed move.

Common operations:
- Changing the deemed positions of some axes/drives, requiring endpoints to be adjusted on owning DDARings, DMs and lastKnownEndpoints.
   Called during initialisation (but no drives are owned)
   Called by G92 (at least the affected drives are owned)
   Called when homing moves change the assumed machine coordinates
   Called when Z probing moves change the assumed machine coordinates
- Retrieving from the DMs endpoints of some drives that were stopped by an endstop or probe, updating the owning DDARing and lastKnownEndpoints, and updating axis corodinates
   Called after a endstop-sensitive or probing move that stops one or more drives, before any adjustment to the assumed machine coordinates
 
Configurations which don't support multiple motion systems:
- For convenience we still maintain lastKnownEndpoints, to reduce the amount of coe that is conditional on supporting multiple motion systems
- We replace MoveState member logicalDrivesOwned by a static constant that means all logical drives