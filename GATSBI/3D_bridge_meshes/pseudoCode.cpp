


std::vector<(x,y,z)> inspected = {}; // inspected voxels
std::vector<(x,y,z)> uninspectedN = {}; // uninspected neighbor voxels (voxels that are uninspected on the same plane)
std::vector<(x,y,z,sidesWONO)> uninspectedT = {}; // uninspected transition voxles (voxels that have all neighbors inspected, but might have a side with no neighbor/obstacle that allows for transition)
                                                  // WONO: without neighbor/obstacle

// Add starting position to inspected
inspected.push_back();
// Add all visible neighbors to uninspectedN and uninspectedT
uninspectedN.push_back();
uninspectedT.push_back();


while(uninspectedN || uninspectedT){
  while(uninspectedN){ // while there are uninspected neighbors
    //Solve GTSP for uninspectedN
    gtspSolver(uninspectedN);
    // Add uninspectedN to inspected
    inspected.push_back();
    // Add new neighbors that are found
    uninspectedN.push_back();
    // Add voxels that have all neighbors inspected, but may have a transition side
    uninspectedT.push_back();
    // TODO: add checker for inspected voxels if they correspond to one within the uninspectedT so we can remove them from uninspectedT
  }

  // Find nearest uninspectedT
  dist2Voxel(currentLocation, uninspectedT, nearestTVoxel);
  // Inspect face of voxel adjacent to the side with no neighbor/occupied voxel
  flyAround(uninspectedT[nearestTVoxel].sidesWONO[0]);
  // Remove side from nearestTVoxel
  uninspectedT[nearestTVoxel].sidesWONO.erase(0);
  if(uninspectedT[nearestTVoxel.sidesWONO.size()] == 0){
    // Add nearestTVoxel to inspected
    inspected.push_back(uninspectedT[nearestTVoxel]);
    uninspectedT.erase(uninspectedT.begin()+nearestTVoxel);
  }
  // Add new neighbors that are found
  uninspectedN.push_back();
  uninspectedT.push_back();

  // TODO: need to figure out exactly how we add to uninspectedT. Should not add sidesWONO if transitioned from.

}
