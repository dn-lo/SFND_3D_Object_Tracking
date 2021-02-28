// Euclidean clustering implementation

#ifndef EUCLIDEANCLUSTER_H_
#define EUCLIDEANCLUSTER_H_

#include <chrono>
#include <vector>
#include <iostream>
// using KD-Tree structure (also include .cpp to help linker since you are including header)
#include "kdTree.h"
#include "kdTree.cpp"

void proximity(int id, std::vector<LidarPoint> &lidarPoints, std::vector<int>& clusterIdx, KdTree* tree, double distanceTol, std::vector<bool> &visitedIds);

std::vector<std::vector<LidarPoint>> euclideanCluster(std::vector<LidarPoint> &lidarPoints, double distanceTol, int minSize, int maxSize);

#endif