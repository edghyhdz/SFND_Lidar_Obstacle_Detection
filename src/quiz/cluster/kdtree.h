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

	/** \brief Helper function to search for nearby nodes in \b *KdTree::root
	  * \param[in] node a node \b *KdTree::root to search for points
	  * \param[in] target x or y value
	  * \param[in] distanceTol distance threshold
	  * \param[in] depth depth
	  * \param[in, out] ids node id
	  */
	void helper_search(Node const *node, std::vector<float> target, float distanceTol, int depth, std::vector<int> *ids){
		std::cout << "Helper search. Depth: " << std::to_string(depth) << std::endl; 

		if (node != NULL){
			std::cout << "Checking (" << node->point[0] << ", " << node->point[1] << ")" << std::endl; 
		} else {
			std::cout << "NULL NODE" << std::endl; 
		}

		if (node != NULL){

			if ( (node->point[0]>=(target[0] - distanceTol)&&node->point[0] <=(target[0]+distanceTol)) && (node->point[1]>=(target[1] - distanceTol)&&node->point[1] <=(target[1]+distanceTol))){

			// if ((((target[0] - distanceTol) <= node->point[0]) && (node->point[0] <= (target[0] + distanceTol))) && (((target[1] - distanceTol) <= node->point[1]) && (node->point[1] <= (target[1] + distanceTol))) ){
				// Calculate distance
				(*ids).push_back(node->id);
			} 
			
			if ((target[depth % 2] - distanceTol) < node->point[depth % 2] ){
				depth++;
				helper_search(node->left, target, distanceTol, depth, ids); 
			}
			if((target[depth % 2] + distanceTol) > node->point[depth % 2]){
				depth++;
				helper_search(node->right, target, distanceTol, depth, ids); 
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		uint32_t depth{0};
		helper_search(root, target, distanceTol, depth, &ids);
		return ids;
	}
	
};




