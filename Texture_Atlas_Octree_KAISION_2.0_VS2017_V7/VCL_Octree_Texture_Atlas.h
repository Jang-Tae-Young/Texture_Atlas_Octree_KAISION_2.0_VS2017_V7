#pragma once
#include "VCL_Voxel_Color_Map.h"
#include "VCL_Voxel_Texture_Atlas_3_Direction.h"
#include "VCL_Voxel_Segmentation.h"
#include "VCL_Voxel_Color_Sequence_Aligner.h"
//#include "Voxel_Slice_Scanned_Data.h"
#include <vector>


class VCL_Octree_Texture_Atlas : public VCL_Voxel_Color_Map,
								 public VCL_Voxel_Segmentation,
								 public VCL_Voxel_Color_Sequence_Aligner
{
public:
	VCL_Octree_Texture_Atlas();
	~VCL_Octree_Texture_Atlas();

	void Set_Parameters(
		bool in_shape_coding_mode_on_off,
		int in_plane_mode_sequence,
		int in_alignment_mode,
		int in_block_size,
		int in_threshold_for_cluster_size,
		float in_threshold_for_segmentation,
		int in_map_width,
		int in_map_height);

	void Generate_Voxel_Map_with_Multiple_Images(
		VCL_DoCube_X_Color *in_docube,
		std::vector<int> in_plane_mode_sequence,
		bool in_shape_coding_mode_on_off,
		int in_alignment_mode,
		int in_block_size,
		int in_encoder,
		CKvSet_of_MatrixUcharRgb *out_voxel_maps,
		CKvSet_of_MatrixBool *out_voxel_map_masks,
		CKvSet_of_MatrixInt *out_voxel_idx_maps,
		CKvMatrixInt *out_list_for_position_N_by_3_second_image);



protected:

	void Set_SubCoord(int in_slice_mode_0_X_1_Y_2_Z) {
		if (in_slice_mode_0_X_1_Y_2_Z == 0) {
			zz_coord0 = 2;
			zz_coord1 = 1;
		}
		else if (in_slice_mode_0_X_1_Y_2_Z == 1) {
			zz_coord0 = 2;
			zz_coord1 = 0;
		}
		else {
			zz_coord0 = 0;
			zz_coord1 = 1;
		}
	}

	void Segmentation_And_Surface_Scanning(
		VCL_DoCube_X_Color *in_docube,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<int>>> &out_voxel_index);

	void Divide_Voxels(
		std::vector<std::vector<std::vector<float>>> &in_segmented_voxels,
		std::vector<std::vector<std::vector<float>>> &in_segmented_colors,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<std::vector<float>>>> &out_segmented_divided_voxels,
		std::vector<std::vector<std::vector<std::vector<float>>>> &out_segmented_divided_colors);

	void Divide_Voxels(
		VCL_DoCube_X_Color *in_docube,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<int>>> &in_voxel_index,
		std::vector<std::vector<std::vector<std::vector<int>>>> &out_divided_voxel_index);

	void Ordering_Texture_on_Codes(
		VCL_DoCube_X_Color *in_docube,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<std::vector<int>>>> &io_divided_Texture_on_Code);

	void Ordering_Texture_on_Code(
		VCL_DoCube_X_Color *in_docube,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<int> &io_ToC);

	void Surface_Scanning(
		std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_voxels,
		std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_colors,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<Voxel_Slice_Scanned_Data>>> &out_scanned_voxel_data);

	void Surface_Scanning(
		std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_voxels,
		std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_colors,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<std::vector<std::vector<std::vector<int>>>> &out_scanned_voxel_data);

	bool gvsi_Get_Voxel_Sequence(
		VCL_DoCube_X_Color *in_docube,
		int &in_plane_mode_sequence,
		std::vector<Voxel_Slice_Scanned_Data> &out_voxel_sequence);

	bool gvsi_Get_New_Voxel_Sequence(
		VCL_DoCube_X_Color *in_docube,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<int> &io_ToC);


	bool s_gvm_Generate_Voxel_Map__Intra_Prediction_Coding(
		std::vector<std::vector<int>> &in_segmented_texture_on_code,
		std::vector<int> &in_offsets,
		CKvMatrixInt *out_voxel_color_idx_map,
		CKvMatrixBool *out_masks);

	void MinMax(
		std::vector<std::vector<float>> &in_voxels,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<float> &outMinMax_0,
		std::vector<float> &outMinMax_1);

	void MinMax(
		VCL_DoCube_X_Color *in_docube,
		std::vector<std::vector<int>> &in_Blob_ToC,
		int in_slice_mode_0_X_1_Y_2_Z,
		std::vector<float> &outMinMax_0,
		std::vector<float> &outMinMax_1);


	bool zz_shape_coding_mode_on_off;
	int zz_plane_mode_sequence;
	int zz_threshold_for_cluster_size;
	int zz_prediction_searching_range;
	int zz_prediction_mode;
	int zz_error_threshold;
	int zz_alignment_mode;

	int zz_coord0, zz_coord1;
	int zz_ww, zz_hh, zz_dd;

};
