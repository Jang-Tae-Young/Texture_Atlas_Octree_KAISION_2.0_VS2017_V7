#ifndef VCL_SPARSE_VOXEL_OCTREE
#define VCL_SPARSE_VOXEL_OCTREE

#include <vector>
#include "VCL_Node.h"

class VCL_Sparse_Voxel_Octree : public VCL_Node
{
public:

	VCL_Sparse_Voxel_Octree();
	~VCL_Sparse_Voxel_Octree();

	void i_Import(VCL_DoCube_X_Color *in_docube);
	bool gsvo_Generate_Sparse_Voxel_Octree_Top_Down();
	bool gsvoc_Generate_Sparse_Voxel_Octree_Top_Down_w_Color();
	bool gsvo_Generate_Sparse_Voxel_Octree_Top_Down_Using_Volume_Data();
	bool gsvo_Generate_Sparse_Voxel_Octree_Bottom_Up(); // Not use

public:
	std::vector<VCL_Node> gsvo_Get_Sparse_Voxel_Octree(){ return zz_nodes; }
	void gsv_Get_Surface_Voxel(CKvSet_of_Point3D *out_voxels);
	void gsv_Get_Surface_Voxel(CKvSet_of_RgbaF *out_voxels_colors);
	void gsv_Get_Surface_Voxel_Colors(CKvVectorUcharRgb *out_voxels_colors);
	void gsvl_Get_Surface_Voxel_In_Level(CKvSet_of_Point3D *out_voxels, int in_level);

	int gms_Get_Memory_size_SVO();
	int gms_Get_Memory_size_SVO_64bit();
	int gms_Get_Memory_size_Only_Child_Mask();
	int gms_Get_Memory_size_String();
	void gspc_Get_Surface_Points_And_Colors(
		CKvDepot_of_Point3Df *out_surface_points,
		CKvDepot_of_RgbaF *out_set_of_color);

public:
	bool ssvo_Save_Sparse_Voxel_Octree(CKvString in_filename, int *out_memory_bit_or_NULL);
	bool ssvo_Save_Sparse_Voxel_Octree_Only_Child_Mask(CKvString in_filename, int *out_memory_bit_or_NULL);
	bool ssvo_Save_Sparse_Voxel_Octree_String(CKvString in_filename, double *out_memory_byte_or_NULL);

protected:
	void gv_Generate_Volume(CKvVolumeBool *out_volume);
	VCL_Node srn_Set_Root_node();	
	VCL_Node srn_Set_Root_node(CKvSet_of_Voxel *in_set_of_voxel);
	VCL_Node srn_Set_Root_node(CKvSet_of_Voxel *in_set_of_voxel, CKvDepot_of_RgbaF *in_set_of_voxel_color);

	CKvVolumeBool gv_Get_Volume(
		CKvVolumeBool *in_voxel_volume,
		int xo, int dx, 
		int yo, int dy, 
		int zo, int dz);	
	
	bool iev_Is_Empty_Volume(CKvVolumeBool *in_volume);
	bool iev_Is_Empty_Volume(
		CKvSet_of_Voxel *in_set_of_voxels,
		int xo, int dx,
		int yo, int dy,
		int zo, int dz);
	bool iev_Is_Empty_Volume(
		std::vector<int> &in_set_of_voxels,
		int xo, int dx,
		int yo, int dy,
		int zo, int dz,
		std::vector<int> &out_set_of_voxels);
	bool iev_Is_Empty_Volume(
		std::vector<int> &in_set_of_voxels,
		std::vector<int> &in_set_of_color_index,
		int xo, int dx,
		int yo, int dy,
		int zo, int dz,
		std::vector<int> &out_set_of_voxels,
		std::vector<int> &out_set_of_color_index);

	int gnc_Get_The_Number_of_Child(char &in_children_mask);
	void gtv_Generate_Test_Volume(CKvVolumeBool *out_volume);
	bool rbm_Read_Bit_Mask(
		char &in_children_mask,
		int in_bit_index);
	void Get_Level_Given_Block_Size(int &in_size, int &out_level);

protected:

	char zz_powers[8];

	VCL_DoCube_X_Color zz_docube;
	CKvVolumeBool zz_voxel_volume;
	int zz_voxel_resolution;
	std::vector<VCL_Node> zz_nodes;
};



#endif