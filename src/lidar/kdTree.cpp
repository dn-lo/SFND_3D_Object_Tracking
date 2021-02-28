// KD-Tree implementation
#include "kdTree.h"

// Tree destructor uses helper fcn destroySubtree and sets root pointer to null
// Needed to avoid memory leak
KdTree::~KdTree() {
	deleteSubtree(root);
	// std::cout << "Deleting root pointer" << std::endl;
	root = nullptr;
}

// Insert new node in tree
void KdTree::insert(std::vector<double> point, int id)
{
	// Create new node using constructor
	Node* node = new Node(point, id);

	// Use recursive insertHelper method to insert node in tree
	// Start from root and at depth 0 (default)
	_insertHelper(root, node);
}

// Return a list of points ids that are within distanceTol from target
std::vector<int> KdTree::search(std::vector<double> target, double distanceTol)
{
	std::vector<int> ids;
	
	// Use recursive searchHelper method
	// Start from root and at depth 0 (default)
	// ids is an empty vector passed by reference and stores the final result
	_searchHelper(root, target, ids, distanceTol);
	return ids;
}

// Helper function for insert method
void KdTree::_insertHelper(Node* &cur, Node* node, uint depth) 
{
	// If current pointer is nullptr, we have found the position to insert our node
	if (!cur) {
		cur = node;
		return;
	}

	// Else compute current dimension: alternate between x, y and z. Then increment tree depth
	uint cd = depth % 3;
	depth++;

	// Insert left if smaller than current node along dimension, right if greater
	if (node->point[cd] < cur->point[cd])
		_insertHelper(cur->left, node, depth);
	else
		_insertHelper(cur->right, node, depth);
}

// Helper function for search method
void KdTree::_searchHelper(Node* &cur, std::vector<double> &target, std::vector<int> &ids, double distanceTol, uint depth) 
{
	// If current node pointer is nullptr, we can exit the function (search is over)
	if (cur) {
		// Else compute current dimension: alternate between x, y and z. Then increment tree depth
		uint cd = depth % 3;
		depth++;

		// Check if distance of current point from target is smaller than distanceTol along the three dimensions
		double deltaX = cur->point[0] - target[0];
		double deltaY = cur->point[1] - target[1];
		double deltaZ = cur->point[2] - target[2];
		if ( ((deltaX <= distanceTol) && (deltaX >= -distanceTol)) && ((deltaY <= distanceTol) && (deltaY >= -distanceTol)) && ((deltaZ <= distanceTol) && (deltaZ >= -distanceTol))) 
		{
			// If so, check if distance is smaller than distanceTol and add current point id to ids
			double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
			if (distance <= distanceTol)
				ids.push_back(cur->id);
		}
		// Search left if box lower boundary is smaller than the current node
		if (target[cd] - distanceTol <= cur->point[cd])
			_searchHelper(cur->left, target, ids, distanceTol, depth);
		// Search right if box upper boundary is greater than the current node
		if (target[cd] + distanceTol >= cur->point[cd])
			_searchHelper(cur->right, target, ids, distanceTol, depth);
	}
}

// Recursively deletes subtree rooted at a specific node 
void KdTree::deleteSubtree(Node* subtreeRootPtr) 
{
	// Exit if we reached end of the tree (pointer is null)
	if (!subtreeRootPtr) 
		return;
	
	// Else use a post-order traversal to delete children nodes first, then our tree
	// Calling this function recursively
	deleteSubtree(subtreeRootPtr->left);
	deleteSubtree(subtreeRootPtr->right);

	// Clear children pointer
	subtreeRootPtr->left = nullptr;
	subtreeRootPtr->right = nullptr;

	// std::cout << "Deleting node " << subtreeRootPtr->id << std::endl;

	// Delete current subtree root node
	delete subtreeRootPtr;
}