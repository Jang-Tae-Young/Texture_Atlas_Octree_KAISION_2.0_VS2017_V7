#pragma once
#include "VCL_Voxel_Color_Map.h"
#include "VCL_Voxel_Texture_Atlas_3_Direction.h"
//#include "VCL_Voxel_Segmentation.h"
//#include "VCL_Voxel_Color_Sequence_Aligner.h"
//#include "Voxel_Slice_Scanned_Data.h"
#include <vector>


class VCL_Octree_Texture_Atlas : public VCL_Voxel_Color_Map,
								 public VCL_Voxel_Texture_Atlas_3_Direction
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
	bool zz_shape_coding_mode_on_off;
	int zz_plane_mode_sequence;
	int zz_threshold_for_cluster_size;
	int zz_prediction_searching_range;
	int zz_prediction_mode;
	int zz_error_threshold;
	int zz_alignment_mode;


};
