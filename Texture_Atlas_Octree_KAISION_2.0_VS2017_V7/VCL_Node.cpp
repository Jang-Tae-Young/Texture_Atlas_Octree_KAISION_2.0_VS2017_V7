#include "stdafx.h"
#include "VCL_Node.h"

//*******************************************************************************************
VCL_Node::VCL_Node()
//*******************************************************************************************
{
	zz_depth_level = 0;
	zz_parent_index = 0;
	Is_Root_or_Leaf_or_Not = 0;
	zz_children_mask = 0;
	zz_start_xyz[0] = zz_start_xyz[1] = zz_start_xyz[2] = 0;

	zz_children_indices[0] = -1;
	zz_children_indices[1] = -1;
	zz_children_indices[2] = -1;
	zz_children_indices[3] = -1;
	zz_children_indices[4] = -1;
	zz_children_indices[5] = -1;
	zz_children_indices[6] = -1;
	zz_children_indices[7] = -1;
	zz_code_string = "(00000000";
}
//*******************************************************************************************
VCL_Node::VCL_Node(const VCL_Node &T)
//*******************************************************************************************
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

	this->zz_red = T.zz_red;
	this->zz_green = T.zz_green;
	this->zz_blue = T.zz_blue;

	//T.zz_code_string.e_Export(string);
	//strcpy((char *)this->zz_code_string, (char *)T.zz_code_string);
	//T.zz_code_string.e_Export(string);
	//this->zz_code_string.cp_Copy(T.zz_code_string);
}
//*******************************************************************************************
VCL_Node::~VCL_Node()
//*******************************************************************************************
{

}
//*******************************************************************************************