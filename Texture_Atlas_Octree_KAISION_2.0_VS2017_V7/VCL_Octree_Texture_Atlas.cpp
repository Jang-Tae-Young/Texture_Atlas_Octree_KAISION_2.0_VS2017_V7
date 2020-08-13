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
	std::vector<std::vector<std::vector<int>>> ToC;
	for (auto mode : in_plane_mode_sequence) {

		Segmentation_And_Surface_Scanning(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			mode,//int in_slice_mode_0_X_1_Y_2_Z,
			ToC);//std::vector<std::vector<std::vector<int>>> &out_voxel_index)

		std::vector<std::vector<std::vector<std::vector<int>>>> divided_voxel_index;
		Divide_Voxels(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			mode,//int in_slice_mode_0_X_1_Y_2_Z,
			ToC,//std::vector<std::vector<std::vector<int>>> &in_voxel_index,
			divided_voxel_index);//std::vector<std::vector<std::vector<std::vector<int>>>> &out_divided_voxel_index)

		Ordering_Texture_on_Codes(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			mode,//int in_slice_mode_0_X_1_Y_2_Z,
			divided_voxel_index);//std::vector<std::vector<std::vector<std::vector<int>>>> &io_divided_Texture_on_Code)
		
		for (int m = 0; m < divided_voxel_index.size(); m++) {
			for (int n = 0; n < divided_voxel_index[m].size(); n++) {
				std::vector<int> circular_shift, offsets;
				atoc_Align_Texture_on_Code(
					in_docube,//VCL_DoCube_X_Color *in_docube,
					in_alignment_mode,//int in_alignment_mode,
					divided_voxel_index[m][n],//std::vector<std::vector<int>> &io_segmented_texture_on_code,
					circular_shift,//std::vector<int> &out_circular_shift,
					offsets);//std::vector<int> &out_offsets)

				CKvMatrixInt colorIdxMap; CKvMatrixBool masks;
				s_gvm_Generate_Voxel_Map__Intra_Prediction_Coding(
					divided_voxel_index[m][n],//std::vector<std::vector<int>> &in_segmented_texture_on_code,
					offsets,//std::vector<int> &in_offsets,
					&colorIdxMap,//CKvMatrixInt *out_voxel_color_idx_map,
					&masks);//CKvMatrixBool *out_masks)



			}
		}
	}





}
//************************************************************************
void VCL_Octree_Texture_Atlas::Segmentation_And_Surface_Scanning(
	VCL_DoCube_X_Color *in_docube,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<int>>> &out_voxel_index)
//************************************************************************
{
	std::vector<Voxel_Slice_Scanned_Data> voxel_sequence;
	gvsi_Get_Voxel_Sequence(
		in_docube,//VCL_DoCube_X_Color *in_docube,
		in_slice_mode_0_X_1_Y_2_Z,//int &in_plane_mode_sequence,
		voxel_sequence);//std::vector<Voxel_Slice_Scanned_Data> &out_voxel_sequence)

	stoc_Segment_Voxel_Slice_Scanned_Data(
		voxel_sequence,//std::vector<Voxel_Slice_Scanned_Data> &in_texture_on_code,
		out_voxel_index);//std::vector<std::vector<std::vector<int>>> &out_segmented_texture_on_code)
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
void VCL_Octree_Texture_Atlas::Divide_Voxels(
	VCL_DoCube_X_Color *in_docube,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<int>>> &in_voxel_index,
	std::vector<std::vector<std::vector<std::vector<int>>>> &out_divided_voxel_index)
//************************************************************************
{
	float **p_point3d = in_docube->gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mp();
	
	for (auto Blob_ToC : in_voxel_index) {

		std::vector<float> MinMax_0, MinMax_1;
		std::vector<std::vector<int>> divided_ToC = std::vector<std::vector<int>>(4);

		MinMax(
			in_docube,//VCL_DoCube_X_Color *in_docube,
			Blob_ToC,//std::vector<std::vector<int>> &in_Blob_ToC,
			in_slice_mode_0_X_1_Y_2_Z,//int in_slice_mode_0_X_1_Y_2_Z,
			MinMax_0,//std::vector<float> &outMinMax_0,
			MinMax_1);//std::vector<float> &outMinMax_1);
		
		auto step_0 = MinMax_0[1] - MinMax_0[0];
		auto step_1 = MinMax_1[1] - MinMax_1[0];
		
		for (auto ToC : Blob_ToC) {
			for (auto Element : ToC) {

				if (p_point3d[Element][zz_coord0] >= MinMax_0[0] &&
					p_point3d[Element][zz_coord0] < MinMax_0[0] + (step_0 / 2) &&
					p_point3d[Element][zz_coord1] >= MinMax_1[0] &&
					p_point3d[Element][zz_coord1] < MinMax_1[0] + (step_1 / 2)) {

					divided_ToC[0].push_back(Element);
				}
				else if (p_point3d[Element][zz_coord0] >= MinMax_0[0] + (step_0 / 2) &&
					p_point3d[Element][zz_coord0] <= MinMax_0[1] &&
					p_point3d[Element][zz_coord1] >= MinMax_1[0] &&
					p_point3d[Element][zz_coord1] < MinMax_1[0] + (step_1 / 2)) {

					divided_ToC[1].push_back(Element);
				}
				else if (p_point3d[Element][zz_coord0] >= MinMax_0[0] &&
					p_point3d[Element][zz_coord0] < MinMax_0[1] + (step_0 / 2) &&
					p_point3d[Element][zz_coord1] >= MinMax_1[0] + (step_1 / 2) &&
					p_point3d[Element][zz_coord1] <= MinMax_1[1]) {
					
					divided_ToC[2].push_back(Element);
				}
				else {
	
					divided_ToC[3].push_back(Element);
				}
			}
		}
	}

}
//************************************************************************
void VCL_Octree_Texture_Atlas::Ordering_Texture_on_Codes(
	VCL_DoCube_X_Color *in_docube,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<std::vector<int>>>> &io_divided_Texture_on_Code)
//************************************************************************
{
	int num_segments   = io_divided_Texture_on_Code.size();
	float **p_point3df = in_docube->gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mp();
	for (int k = 0; k < io_divided_Texture_on_Code.size(); k++) {
		for (int l = 0; l < io_divided_Texture_on_Code[k].size(); l++) {
			if (io_divided_Texture_on_Code[k][l].size() != 0) {
				for (int m = 0; m < io_divided_Texture_on_Code[k][l].size(); m++) {
					Ordering_Texture_on_Code(
						in_docube,//VCL_DoCube_X_Color *in_docube,
						in_slice_mode_0_X_1_Y_2_Z,//int in_slice_mode_0_X_1_Y_2_Z,
						io_divided_Texture_on_Code[k][l][m]);//std::vector<int> &io_ToC);
				}
			}
		}
	}
}
//************************************************************************
void VCL_Octree_Texture_Atlas::Ordering_Texture_on_Code(
	VCL_DoCube_X_Color *in_docube,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<int> &io_ToC)
//************************************************************************
{
	float **p_point3df = in_docube->gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mp();
	float *prevPoint3D = p_point3df[0];
	for (int n = 1; n < io_ToC.size(); n++) {
		float *curPoint3D = p_point3df[io_ToC[n]];
		if ((abs(prevPoint3D[0] - curPoint3D[0]) > 1.f) ||
			(abs(prevPoint3D[1] - curPoint3D[1]) > 1.f) ||
			(abs(prevPoint3D[2] - curPoint3D[2]) > 1.f)) {

			gvsi_Get_New_Voxel_Sequence(
				in_docube,//VCL_DoCube_X_Color *in_docube,
				in_slice_mode_0_X_1_Y_2_Z,//int in_slice_mode_0_X_1_Y_2_Z,
				io_ToC);//std::vector<int> &io_ToC)

			return;
		}
		else{
			prevPoint3D = curPoint3D;
		}
	}
}
//************************************************************************
void VCL_Octree_Texture_Atlas::Surface_Scanning(
	std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_voxels,
	std::vector<std::vector<std::vector<std::vector<float>>>> &in_segmented_divided_colors,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<Voxel_Slice_Scanned_Data>>> &out_scanned_voxel_data)
//************************************************************************
{
	std::vector<std::vector<Voxel_Slice_Scanned_Data>> set_tmp;
	std::vector<Voxel_Slice_Scanned_Data> tmp;
	int idx_segment, idx_divide;
	idx_segment = 0;
	for (auto segment : in_segmented_divided_voxels) {

		idx_divide = 0;
		for (auto divide : segment) {
			VCL_DoCube_X_Color docube(
				divide, 
				in_segmented_divided_colors[idx_segment][idx_divide],
				zz_ww,
				zz_hh,
				zz_dd);

			gvsi_Get_Voxel_Sequence(
				&docube,//VCL_DoCube_X_Color *in_docube,
				in_slice_mode_0_X_1_Y_2_Z,//int &in_plane_mode_sequence,
				tmp);//std::vector<Voxel_Slice_Scanned_Data> &out_voxel_sequence)

			set_tmp.push_back(tmp);
			idx_divide++;
		}
		out_scanned_voxel_data.push_back(set_tmp);
		idx_segment++;
	}

}
//************************************************************************
void VCL_Octree_Texture_Atlas::Surface_Scanning(
	std::vector<std::vector<std::vector<std::vector<float>>>> &io_segmented_divided_voxels,
	std::vector<std::vector<std::vector<std::vector<float>>>> &io_segmented_divided_colors,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<std::vector<std::vector<std::vector<int>>>> &out_scanned_voxel_data)
//************************************************************************
{
	std::vector<Voxel_Slice_Scanned_Data> tmp;
	int idx_segment, idx_divide;
	int ww, num_voxels;
	int num_segments, num_divide;

	idx_segment = 0;
	num_segments = io_segmented_divided_voxels.size();
	for (int k = 0; k < num_segments; k++) {
		std::vector<std::vector<std::vector<int>>> divided_idx;
		num_divide = io_segmented_divided_voxels[k].size();
		for (int m = 0; m < num_divide; m++) {

			VCL_DoCube_X_Color docube(
				io_segmented_divided_voxels[k][m],
				io_segmented_divided_colors[k][m],
				zz_ww,
				zz_hh,
				zz_dd);

			gvsi_Get_Voxel_Sequence(
				&docube,//VCL_DoCube_X_Color *in_docube,
				in_slice_mode_0_X_1_Y_2_Z,//int &in_plane_mode_sequence,
				tmp);//std::vector<Voxel_Slice_Scanned_Data> &out_voxel_sequence)

			std::vector<std::vector<int>> set_tmp;
			for (auto scanned_data : tmp) {
				if (scanned_data.voxel_idx_sequences.size() != 0) {
					assert(scanned_data.voxel_idx_sequences.size() > 1);
					set_tmp.push_back(scanned_data.voxel_idx_sequences[0]);
				}
			}

			float **p_point3d = docube.gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mps(ww, num_voxels);
			float **p_color   = docube.gvc_Get_Voxel_Color().gps_Get_Pointer_of_Space()->mp();

			for (int n = 0; n < num_voxels; n++) {

				io_segmented_divided_voxels[k][m][n][0] = p_point3d[n][0];
				io_segmented_divided_voxels[k][m][n][1] = p_point3d[n][1];
				io_segmented_divided_voxels[k][m][n][2] = p_point3d[n][2];

				io_segmented_divided_colors[k][m][n][0] = p_color[n][0];
				io_segmented_divided_colors[k][m][n][1] = p_color[n][1];
				io_segmented_divided_colors[k][m][n][2] = p_color[n][2];
			}

			divided_idx.push_back(set_tmp);
		}
		out_scanned_voxel_data.push_back(divided_idx);
	}

}
//************************************************************************
bool VCL_Octree_Texture_Atlas::gvsi_Get_Voxel_Sequence(
	VCL_DoCube_X_Color *in_docube,
	int &in_plane_mode_sequence,
	std::vector<Voxel_Slice_Scanned_Data> &out_voxel_sequence)
/**********************************************************************************/
{
	JANG_Xrunset3dShort xrunset3d;
	int ww, hh, dd, k, num_plane_mode = 1, num_p;

	in_docube->gr_Get_Resolution(ww, hh, dd);
	xrunset3d.jimport(&in_docube->e_Export_Xvectors_Boundary_Point(), dd);

	num_p = in_docube->gsp_Get_Surface_Points_Pointer()->ne_Number_of_Elements();

	CKvStopWatch sw;
	sw.c_Create(1); sw.r_Reset(0);
	int num_scanned = 0;
	out_voxel_sequence = std::vector<Voxel_Slice_Scanned_Data>(ww);

	if (Get_Surface_Voxel_Indices(
		in_docube,//VCL_DoCube_X_Color *in_docube,
		&xrunset3d,//CKvXrunset3dShort *io_docube,
		in_plane_mode_sequence,//int in_plane_mode,
		ww,//int &in_ww,
		hh,//int &in_hh,
		dd,//int &in_dd,
		&out_voxel_sequence))//std::vector<Voxel_Slice_Scanned_Data> *out_voxel_sequence)
	{
		num_scanned++;
	}

	if (num_scanned != num_plane_mode)
	{
		out_voxel_sequence.erase(out_voxel_sequence.begin() + num_scanned, out_voxel_sequence.end());
	}

	printf("		[Get Scanned Voxel Colors] -> %lf\n", sw.get_Get_Elapsed_Time(0));


	return true;
}
//************************************************************************
bool VCL_Octree_Texture_Atlas::gvsi_Get_New_Voxel_Sequence(
	VCL_DoCube_X_Color *in_docube,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<int> &io_ToC)
//************************************************************************
{
	CKvSdkimChainCode sdkim_chain_code;
	CKvSdkimRunCode runcode;
	CKvXrunsetShort xrunset;
	CKvMatrixBool matrixBool;
	CKvXvectorUchar chain_codes;
	CKvPixelShort FIRST_point_in_the_chain_codes;
	float **p_point3df = in_docube->gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mp();


	bool **pMat = matrixBool.c_Create(zz_hh, zz_ww, false);
	for (auto Element : io_ToC) {
		int x = p_point3df[Element][zz_coord0];
		int y = p_point3df[Element][zz_coord1];
		pMat[y][x] = true;
	}

	xrunset.im_Import(matrixBool);
	runcode.im_Import(xrunset, true);
	sdkim_chain_code.im_Import(runcode);

	if (sdkim_chain_code.no_Number_of_Objects() > 1) {
		assert("Get New Voxel Sequence -> The number of Objects is larger than '1'.\n");
	}

	sdkim_chain_code.gspb_Get_Set_of_Points_on_a_Boundary(
		true,//bool in_mode_for_external_boundary,
		0,//int in_boundary_index,
		NULL,//CKvXvectorShort *out_CORNER_points___or_NULL,
		NULL,//CKvXvectorShort *out_RUN_points___or_NULL,
		NULL,//CKvXvectorShort *out_ALL_BOUNDARY_points___or_NULL,
		&chain_codes,//CKvXvectorUchar *out_chain_codes___or_NULL,
		&FIRST_point_in_the_chain_codes,//CKvPixelShort *out_FIRST_point_in_the_chain_codes___or_NULL,
		NULL);//int *out_object_index_related_to_the_boundary___or_NULL);

	CKvXvectorShort b_pt;
	gps_Get_Pixel_String(
		&FIRST_point_in_the_chain_codes,//CKvPixelShort *in_first_pixel,
		&chain_codes,//CKvXvectorUchar *in_chain_code,
		&b_pt);//CKvXvectorShort *out_boundary_pixel)

	short *p_b_pt;
	b_pt.z_gii_Get_Informations_Internal(NULL, &p_b_pt, NULL, NULL);
	int num_pt = b_pt.ne();

	int element_count = 0;
	int voxel_idx;
	std::vector<int> tmp;
	for (int l = 0; l < num_pt; l += 2)
	{
		/******************************************************************/
		//Get Voxel Index Vector
		if (in_slice_mode_0_X_1_Y_2_Z == 0)
		{
			voxel_idx = in_docube->gvi_Get_Voxel_Index(
				p_point3df[0][0],
				p_b_pt[l + 1],
				p_b_pt[l]);
		}
		else if (in_slice_mode_0_X_1_Y_2_Z == 1)
		{
			voxel_idx = in_docube->gvi_Get_Voxel_Index(
				p_b_pt[l + 1],
				p_point3df[0][1],
				p_b_pt[l]);
		}
		else if (in_slice_mode_0_X_1_Y_2_Z == 2)
		{
			voxel_idx = in_docube->gvi_Get_Voxel_Index(
				p_b_pt[l],
				p_b_pt[l + 1],
				p_point3df[0][2]);
		}

		if (voxel_idx == -1)
		{
			assert("[[Error : Invalid Voxel Index]]\n");
			return false;
		}
		/******************************************************************/
		tmp.push_back(voxel_idx);
	}

	io_ToC = tmp;
}
//************************************************************************
bool VCL_Octree_Texture_Atlas::s_gvm_Generate_Voxel_Map__Intra_Prediction_Coding(
	std::vector<std::vector<int>> &in_segmented_texture_on_code,
	std::vector<int> &in_offsets,
	CKvMatrixInt *out_voxel_color_idx_map,
	CKvMatrixBool *out_masks)
//************************************************************************
{
	if (in_segmented_texture_on_code.size() != in_offsets.size()) {
		assert("Error!");
		return false;
	}


	int num_toc = in_segmented_texture_on_code.size();
	int max_length = -1;
	for (int l = 0; l < num_toc; l++)
	{
		int tmp = in_segmented_texture_on_code[l].size() + in_offsets[l];
		if (max_length < tmp)
		{
			max_length = tmp;
		}
	}

	int **p_color_map_idx = out_voxel_color_idx_map->c_Create(num_toc, max_length, -1);
	bool **p_mask         = out_masks->c_Create(num_toc, max_length, false);

	for (int l = 0; l < num_toc; l++){
		int num_element = in_segmented_texture_on_code[l].size();
		for (int j = 0; j < num_element; j++){
			p_color_map_idx[l][j + in_offsets[l]] = (int)in_segmented_texture_on_code[l][j];
			if ((int)in_segmented_texture_on_code[l][j] != -1){
				p_mask[l][j + in_offsets[l]] = true;
			}
		}
	}


	return true;
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
			min1 = aVoxel[zz_coord1];
		}
		if (aVoxel[zz_coord1] > max1) {
			max1 = aVoxel[zz_coord1];
		}
	}

	outMinMax_0.push_back(min0);
	outMinMax_0.push_back(max0);
	
	outMinMax_1.push_back(min1);
	outMinMax_1.push_back(max1);
}
//************************************************************************
void VCL_Octree_Texture_Atlas::MinMax(
	VCL_DoCube_X_Color *in_docube,
	std::vector<std::vector<int>> &in_Blob_ToC,
	int in_slice_mode_0_X_1_Y_2_Z,
	std::vector<float> &outMinMax_0,
	std::vector<float> &outMinMax_1)
//************************************************************************
{
	float min0, max0, min1, max1;
	int dum, num_points;

	float **p_point3d = in_docube->gsp_Get_Surface_Points_Pointer()->gps_Get_Pointer_of_Space()->mps(dum,num_points);

	min0 = min1 = FLT_MAX;
	max0 = max1 = FLT_MIN;

	for (auto ToC : in_Blob_ToC) {
		for (auto Element : ToC) {

			if (p_point3d[Element][zz_coord0] < min0) {
				min0 = p_point3d[Element][zz_coord0];
			}
			if (p_point3d[Element][zz_coord0] < max0) {
				max0 = p_point3d[Element][zz_coord0];
			}

			if (p_point3d[Element][zz_coord1] < min1) {
				min1 = p_point3d[Element][zz_coord1];
			}
			if (p_point3d[Element][zz_coord1] < max1) {
				max1 = p_point3d[Element][zz_coord1];
			}

		}
	}
	
	outMinMax_0.push_back(min0);
	outMinMax_0.push_back(max0);

	outMinMax_1.push_back(min1);
	outMinMax_1.push_back(max1);
}
//************************************************************************








