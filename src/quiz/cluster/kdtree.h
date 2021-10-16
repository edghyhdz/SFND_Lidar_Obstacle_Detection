/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		helper_insert(&root, point, 0, id); 
	}

	/** \brief Helper function to add nodes into \b *KdTree::root
	  * \param[in, out] node a node to be added into \b *KdTree::root
	  * \param[in] data x or y value
	  * \param[in] depth x or y split index
	  * \param[in] id node id
	  */
	void helper_insert(Node **node, std::vector<float> data, int depth, int id){
		if (*node == NULL) {
			*node = new Node(data, id); 
		}
		else if (data[depth % 2] < (*node)->point[depth % 2]){
			helper_insert(&(*node)->left, data, depth + 1, id); 
		}
		else {
			helper_insert(&(*node)->right, data, depth + 1, id); 
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




