#ifndef VCL_NODE
#define VCL_NODE

#include <vector>

#define VCL_ROOT_NODE -1
#define VCL_LEAF_NODE 1
#define VCL_NOT_LEAF_NODE 0

class VCL_Node
{
public:

	VCL_Node();
	VCL_Node(const VCL_Node &T);
	~VCL_Node();

	char zz_depth_level; //start depth : 0
	uint64_t zz_start_xyz[3];
	uint64_t zz_parent_index;
	char zz_parent_zz_children_maks_pose;
	char Is_Root_or_Leaf_or_Not;
	char zz_children_mask;
	uint64_t zz_children_indices[8];
	CString zz_code_string;
	std::vector<int> zz_set_of_voxel;
	std::vector<int> zz_color_index;
	unsigned char zz_red, zz_green, zz_blue;

	//*******************************************************************************************	
	bool operator < (const VCL_Node &node2)
	//*******************************************************************************************
	{
		for (int i = 0; i < 8; i++)
		{
			if (this->zz_children_indices[i] == node2.zz_children_indices[i]) continue;
			return this->zz_children_indices[i] < node2.zz_children_indices[i];
		}
		return false;
	};
	//*******************************************************************************************
	void cp_Copy(VCL_Node &T)
	{
		CString string;

		this->zz_depth_level = T.zz_depth_level;
		this->zz_parent_index = T.zz_parent_index;
		this->zz_parent_zz_children_maks_pose = T.zz_parent_zz_children_maks_pose;
		this->Is_Root_or_Leaf_or_Not = T.Is_Root_or_Leaf_or_Not;
		this->zz_children_mask = T.zz_children_mask;
		this->zz_start_xyz[0] = T.zz_start_xyz[0];
		this->zz_start_xyz[1] = T.zz_start_xyz[1];
		this->zz_start_xyz[2] = T.zz_start_xyz[2];

		this->zz_children_indices[0] = T.zz_children_indices[0];
		this->zz_children_indices[1] = T.zz_children_indices[1];
		this->zz_children_indices[2] = T.zz_children_indices[2];
		this->zz_children_indices[3] = T.zz_children_indices[3];
		this->zz_children_indices[4] = T.zz_children_indices[4];
		this->zz_children_indices[5] = T.zz_children_indices[5];
		this->zz_children_indices[6] = T.zz_children_indices[6];
		this->zz_children_indices[7] = T.zz_children_indices[7];

		this->zz_code_string = T.zz_code_string;
		this->zz_set_of_voxel = T.zz_set_of_voxel;
		this->zz_color_index = T.zz_color_index;

		this->zz_red   = T.zz_red;
		this->zz_green = T.zz_green;
		this->zz_blue  = T.zz_blue;

	}
	//*******************************************************************************************

};

#endif