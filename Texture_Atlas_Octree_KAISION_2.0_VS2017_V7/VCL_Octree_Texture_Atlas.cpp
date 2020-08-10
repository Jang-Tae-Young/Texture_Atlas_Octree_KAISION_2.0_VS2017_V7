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
	//sp_Set_Parameter_for_Segmentation(in_threshold_for_segmentation);
	//VCL_Voxel_Texture_Atlas_3_Direction::sp_Set_Parameter_Map_Size(in_map_width, in_map_height);

	//zz_shape_coding_mode_on_off = in_shape_coding_mode_on_off;
	//zz_threshold_for_cluster_size = in_threshold_for_cluster_size;
	//VCL_Voxel_Texture_Atlas_3_Direction::zz_alignment_mode = in_alignment_mode;
	//VCL_Voxel_Texture_Atlas_3_Direction::zz_block_size = in_block_size;
	//VCL_Voxel_Texture_Atlas_3_Direction::zz_plane_mode_sequence = in_plane_mode_sequence;
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



	/*******************************************************************************************/
	//int num_plane_mode = in_plane_mode_sequence.size();
	std::vector<std::vector<std::vector<float>>> voxels, colors;


	for (auto mode : in_plane_mode_sequence)
	{
		Set_SubCoord(mode);

		// Step1) Input voxel partitioning. 
		svd_Segmentation_of_Voxel_Data_along_a_Slice(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			mode,//int in_slice_mode_0_X_1_Y_2_Z,
			voxels,//std::vector<std::vector<std::vector<float>>> &out_voxel_segments,
			colors);//std::vector<std::vector<std::vector<float>>> &out_voxel_colors);

		// Step2) Partitioned voxels -> Octree ? Quadtree ?.
		




		// Step3) Partitioned voxels + Quad -> Texture Maps.


	}





	/*******************************************************************************************/



	//std::vector<bool> voxel_flag;
	//CKvSet_of_MatrixInt    set_of_voxel_map_idx;
	//CKvSet_of_MatrixBool   set_of_voxel_map_idx_mask;
	//CKvDepot_of_MatrixInt  set_of_large_index_maps, set_of_small_index_maps, depot_of_large_index_maps, depot_of_small_index_maps;
	//CKvDepot_of_MatrixBool set_of_large_masks, set_of_small_masks, depot_of_large_masks, depot_of_small_masks;

	//CKvMatrixInt position_of_segmented_voxel_map;
	//CKvMatrixUcharRgb *voxel_map;
	//CKvMatrixInt *voxel_map_idx;
	//CKvMatrixBool *mask, Max_mask;
	//int num_plane_mode = in_plane_mode_sequence.size();
	//int k, num_voxels;

	//voxel_map = out_voxel_maps->c_Create(2);
	//mask = out_voxel_map_masks->c_Create(2);
	//voxel_map_idx = out_voxel_idx_maps->c_Create(2);

	//num_voxels = in_docube->gsp_Get_Surface_Points_Pointer()->ne_Number_of_Elements();

	//CKvStopWatch sw;
	//sw.c_Create(1);
	//sw.r_Reset(0);

	//voxel_flag = std::vector<bool>(num_voxels, false);
	//depot_of_large_index_maps.in_Initialize(); depot_of_small_index_maps.in_Initialize();
	//depot_of_large_masks.in_Initialize();      depot_of_small_masks.in_Initialize();
	//for (k = 0; k < num_plane_mode; k++)
	//{
	//	Partitioning_Voxels_And_Generate_Voxel_Maps(
	//		in_docube,//VCL_DoCube_X_Color *in_docube,
	//		in_plane_mode_sequence[k],//int in_plane_mode_sequence,
	//		zz_alignment_mode,//int in_alignment_mode,
	//		&set_of_voxel_map_idx,//CKvSet_of_MatrixInt *out_set_of_voxel_map_idx,
	//		&set_of_voxel_map_idx_mask);//CKvSet_of_MatrixBool *out_set_of_voxel_map_idx_mask)

	//	printf("k=%d/%d) [Extract_Voxel_Patch_Maps]\n", k, num_plane_mode);
	//	Extract_Voxel_Patch_Maps(
	//		&set_of_voxel_map_idx,//CKvSet_of_MatrixInt *in_set_of_voxel_maps,
	//		&set_of_voxel_map_idx_mask,//CKvSet_of_MatrixBool *in_set_of_voxel_map_masks,
	//		voxel_flag,//std::vector<bool> &in_voxel_flag,
	//		1,//int in_closing_size,
	//		&set_of_large_index_maps,//CKvDepot_of_MatrixInt *out_set_of_large_index_maps,
	//		&set_of_large_masks,//CKvDepot_of_MatrixBool *out_set_of_large_masks,
	//		&set_of_small_index_maps,//CKvDepot_of_MatrixInt *out_set_of_small_index_maps,
	//		&set_of_small_masks);//CKvDepot_of_MatrixBool *out_set_of_small_masks);

	//	depot_of_large_index_maps.ap_Append(false, &set_of_large_index_maps, NULL, NULL);
	//	depot_of_small_index_maps.ap_Append(false, &set_of_small_index_maps, NULL, NULL);
	//	depot_of_large_masks.ap_Append(false, &set_of_large_masks, NULL, NULL);
	//	depot_of_small_masks.ap_Append(false, &set_of_small_masks, NULL, NULL);
	//}



}
//************************************************************************
void VCL_Octree_Texture_Atlas::Divide_Voxels(
	std::vector<std::vector<std::vector<float>>> &in_segmented_voxels,
	std::vector<std::vector<std::vector<float>>> &in_segmented_colors,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<std::vector<float>>>> &out_segmented_divided_voxels,
	std::vector<std::vector<std::vector<std::vector<float>>>> &out_segmented_divided_colors)
//************************************************************************
{
	int num_segment = 0;
	for (auto voxels : in_segmented_voxels) {
	
		std::vector<float> MinMax_0, MinMax_1;
		std::vector<std::vector<std::vector<float>>> divided_voxels = std::vector<std::vector<std::vector<float>>>(4);
		std::vector<std::vector<std::vector<float>>> divided_colors = std::vector<std::vector<std::vector<float>>>(4);


		MinMax(
			voxels,//std::vector<std::vector<float>> &in_voxels,
			in_slice_mode_0_X_1_Y_2_Z,//int in_slice_mode_0_X_1_Y_2_Z,
			MinMax_0,//std::vector<float> &outMinMax_0,
			MinMax_1);//std::vector<float> &outMinMax_1)
		
		auto step_0 = MinMax_0[1] - MinMax_0[0];
		auto step_1 = MinMax_1[1] - MinMax_1[0];
		int count = 0;
		for (auto aVoxel : voxels){

			if (aVoxel[zz_coord0] >= MinMax_0[0] &&
				aVoxel[zz_coord0] < MinMax_0[0] + (step_0 / 2) &&
				aVoxel[zz_coord1] >= MinMax_1[0] &&
				aVoxel[zz_coord1] < MinMax_1[0] + (step_1 / 2)) {

				divided_voxels[0].push_back(aVoxel);
				divided_colors[0].push_back(in_segmented_colors[num_segment][count]);


			}
			else if (aVoxel[zz_coord0] >= MinMax_0[0] + (step_0/2) &&
					aVoxel[zz_coord0] <= MinMax_0[1] &&
					aVoxel[zz_coord1] >= MinMax_1[0] &&
					aVoxel[zz_coord1] < MinMax_1[0] + (step_1 / 2)) {

				divided_voxels[1].push_back(aVoxel);
				divided_colors[1].push_back(in_segmented_colors[num_segment][count]);

			}
			else if (aVoxel[zz_coord0] >= MinMax_0[0] &&
					aVoxel[zz_coord0] < MinMax_0[1] + (step_0/2) &&
					aVoxel[zz_coord1] >= MinMax_1[0] + (step_1/2) &&
					aVoxel[zz_coord1] <= MinMax_1[1]) {

				divided_voxels[2].push_back(aVoxel);
				divided_colors[2].push_back(in_segmented_colors[num_segment][count]);

			}
			else {
				divided_voxels[3].push_back(aVoxel);
				divided_colors[3].push_back(in_segmented_colors[num_segment][count]);
			}
			count++;
		}
		num_segment++;
	}




}
//************************************************************************
void VCL_Octree_Texture_Atlas::MinMax(
	std::vector<std::vector<float>> &in_voxels,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<float> &outMinMax_0,
	std::vector<float> &outMinMax_1)
//************************************************************************
{
	float min0, max0, min1, max1;

	min0 = min1 = FLT_MAX;
	max0 = max1 = FLT_MIN;

	for (auto aVoxel : in_voxels){

		if (aVoxel[zz_coord0] < min0) { 
			min0 = aVoxel[zz_coord0];
		}
		if (aVoxel[zz_coord0] > max0) {
			max0 = aVoxel[zz_coord0];
		}
	
		if (aVoxel[zz_coord1] < min1) {
			min1 = aVoxel[zz_coord0];
		}
		if (aVoxel[zz_coord1] > max1) {
			max1 = aVoxel[zz_coord0];
		}
	}

	outMinMax_0.push_back(min0);
	outMinMax_0.push_back(max0);
	
	outMinMax_1.push_back(min1);
	outMinMax_1.push_back(max1);
}
//************************************************************************









