#include "stdafx.h"
#include "VCL_Octree_Texture_Atlas.h"

/******************************************************************************/
VCL_Octree_Texture_Atlas::VCL_Octree_Texture_Atlas()
/******************************************************************************/
{

}
/******************************************************************************/
VCL_Octree_Texture_Atlas::~VCL_Octree_Texture_Atlas()
/******************************************************************************/
{

}
/******************************************************************************/
void VCL_Octree_Texture_Atlas::Set_Parameters(
	bool in_shape_coding_mode_on_off,
	int in_plane_mode_sequence,
	int in_alignment_mode,
	int in_block_size,
	int in_threshold_for_cluster_size,
	float in_threshold_for_segmentation,
	int in_map_width,
	int in_map_height)
//************************************************************************
{
	sp_Set_Parameter_for_Segmentation(in_threshold_for_segmentation);
	VCL_Voxel_Texture_Atlas_3_Direction::sp_Set_Parameter_Map_Size(in_map_width, in_map_height);

	zz_shape_coding_mode_on_off = in_shape_coding_mode_on_off;
	zz_threshold_for_cluster_size = in_threshold_for_cluster_size;
	VCL_Voxel_Texture_Atlas_3_Direction::zz_alignment_mode = in_alignment_mode;
	VCL_Voxel_Texture_Atlas_3_Direction::zz_block_size = in_block_size;
	VCL_Voxel_Texture_Atlas_3_Direction::zz_plane_mode_sequence = in_plane_mode_sequence;

}
//************************************************************************
void VCL_Octree_Texture_Atlas::Generate_Voxel_Map_with_Multiple_Images(
	VCL_DoCube_X_Color *in_docube,
	std::vector<int> in_plane_mode_sequence,
	bool in_shape_coding_mode_on_off,
	int in_alignment_mode,
	int in_block_size,
	int in_encoder,
	CKvSet_of_MatrixUcharRgb *out_voxel_maps,
	CKvSet_of_MatrixBool *out_voxel_map_masks,
	CKvSet_of_MatrixInt *out_voxel_idx_maps,
	CKvMatrixInt *out_list_for_position_N_by_3_second_image)
//************************************************************************
{
	std::vector<bool> voxel_flag;
	CKvSet_of_MatrixInt    set_of_voxel_map_idx;
	CKvSet_of_MatrixBool   set_of_voxel_map_idx_mask;
	CKvDepot_of_MatrixInt  set_of_large_index_maps, set_of_small_index_maps, depot_of_large_index_maps, depot_of_small_index_maps;
	CKvDepot_of_MatrixBool set_of_large_masks, set_of_small_masks, depot_of_large_masks, depot_of_small_masks;

	CKvMatrixInt position_of_segmented_voxel_map;
	CKvMatrixUcharRgb *voxel_map;
	CKvMatrixInt *voxel_map_idx;
	CKvMatrixBool *mask, Max_mask;
	int num_plane_mode = in_plane_mode_sequence.size();
	int k, num_voxels;

	voxel_map = out_voxel_maps->c_Create(2);
	mask = out_voxel_map_masks->c_Create(2);
	voxel_map_idx = out_voxel_idx_maps->c_Create(2);

	num_voxels = in_docube->gsp_Get_Surface_Points_Pointer()->ne_Number_of_Elements();

	CKvStopWatch sw;
	sw.c_Create(1);
	sw.r_Reset(0);

	voxel_flag = std::vector<bool>(num_voxels, false);
	depot_of_large_index_maps.in_Initialize(); depot_of_small_index_maps.in_Initialize();
	depot_of_large_masks.in_Initialize();      depot_of_small_masks.in_Initialize();
	for (k = 0; k < num_plane_mode; k++)
	{
		Partitioning_Voxels_And_Generate_Voxel_Maps(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			in_plane_mode_sequence[k],//int in_plane_mode_sequence,
			zz_alignment_mode,//int in_alignment_mode,
			&set_of_voxel_map_idx,//CKvSet_of_MatrixInt *out_set_of_voxel_map_idx,
			&set_of_voxel_map_idx_mask);//CKvSet_of_MatrixBool *out_set_of_voxel_map_idx_mask)

		printf("k=%d/%d) [Extract_Voxel_Patch_Maps]\n", k, num_plane_mode);
		Extract_Voxel_Patch_Maps(
			&set_of_voxel_map_idx,//CKvSet_of_MatrixInt *in_set_of_voxel_maps,
			&set_of_voxel_map_idx_mask,//CKvSet_of_MatrixBool *in_set_of_voxel_map_masks,
			voxel_flag,//std::vector<bool> &in_voxel_flag,
			1,//int in_closing_size,
			&set_of_large_index_maps,//CKvDepot_of_MatrixInt *out_set_of_large_index_maps,
			&set_of_large_masks,//CKvDepot_of_MatrixBool *out_set_of_large_masks,
			&set_of_small_index_maps,//CKvDepot_of_MatrixInt *out_set_of_small_index_maps,
			&set_of_small_masks);//CKvDepot_of_MatrixBool *out_set_of_small_masks);

		depot_of_large_index_maps.ap_Append(false, &set_of_large_index_maps, NULL, NULL);
		depot_of_small_index_maps.ap_Append(false, &set_of_small_index_maps, NULL, NULL);
		depot_of_large_masks.ap_Append(false, &set_of_large_masks, NULL, NULL);
		depot_of_small_masks.ap_Append(false, &set_of_small_masks, NULL, NULL);
	}









}
//************************************************************************
