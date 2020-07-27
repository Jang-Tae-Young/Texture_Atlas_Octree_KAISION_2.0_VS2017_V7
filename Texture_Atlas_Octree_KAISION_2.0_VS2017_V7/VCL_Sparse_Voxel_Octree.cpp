#include "stdafx.h"
#include "VCL_Sparse_Voxel_Octree.h"


//*******************************************************************************************
VCL_Sparse_Voxel_Octree::VCL_Sparse_Voxel_Octree()
//*******************************************************************************************
{
	zz_powers[0] = 1;
	zz_powers[1] = 2;
	zz_powers[2] = 4;
	zz_powers[3] = 8;
	zz_powers[4] = 16;
	zz_powers[5] = 32;
	zz_powers[6] = 64;
	zz_powers[7] = 128;

}
//*******************************************************************************************
VCL_Sparse_Voxel_Octree::~VCL_Sparse_Voxel_Octree()
//*******************************************************************************************
{

}
//*******************************************************************************************
void VCL_Sparse_Voxel_Octree::i_Import(VCL_DoCube_X_Color *in_docube)
//*******************************************************************************************
{
	CKvSet2d_of_XvectorShort surface_rl;
	int dum1, dum2;

	zz_docube.VCL_DoCube::cp_Copy(in_docube);
	//zz_docube.i_Import_Color_Vector(&in_docube->gvc_Get_Voxel_Color());
	zz_docube.cp_Copy_Data(*in_docube);
	zz_docube.gr_Get_Resolution(zz_voxel_resolution,dum1,dum2);

	if (zz_nodes.size() != 0){zz_nodes.clear();}
}
//***********************************************************************************************************
bool VCL_Sparse_Voxel_Octree::gsvo_Generate_Sparse_Voxel_Octree_Top_Down_Using_Volume_Data()
//***********************************************************************************************************
{
	VCL_Node node; CKvVolumeBool tmp;
	CString no_one;
	int max_depth, jump_step, previous_jump_step, k, xx, yy, zz, xxo, yyo, zzo;
	int num_Current_Level_Nodes, num_Previous_Level_Nodes, Current_Depth_Level, Start_Index_of_Previous_Level_Nodes;
	int bit_location;

	gv_Generate_Volume(&zz_voxel_volume);
	max_depth = log2(zz_voxel_resolution);

	//printf("max depth level = %d\n", max_depth);
	//printf("[Set 0th depth]\n");
	node = srn_Set_Root_node();
	zz_nodes.push_back(node);

	Start_Index_of_Previous_Level_Nodes = 0;  no_one = "1";
	num_Current_Level_Nodes = Current_Depth_Level = 1;
	while(1)
	{
		num_Previous_Level_Nodes = num_Current_Level_Nodes;
		Start_Index_of_Previous_Level_Nodes = zz_nodes.size() - num_Previous_Level_Nodes;
		num_Current_Level_Nodes = 0;

		jump_step          = zz_voxel_resolution / pow(2, Current_Depth_Level);
		previous_jump_step = zz_voxel_resolution / pow(2, Current_Depth_Level-1);

		for (k = Start_Index_of_Previous_Level_Nodes; k < Start_Index_of_Previous_Level_Nodes + num_Previous_Level_Nodes; k++)
		{
			xxo = zz_nodes[k].zz_start_xyz[0]; 
			yyo = zz_nodes[k].zz_start_xyz[1]; 
			zzo = zz_nodes[k].zz_start_xyz[2];
			bit_location = 0;
			for (zz = zzo; zz < zzo + previous_jump_step ; zz = zz + jump_step)
			{
				for (yy = yyo; yy < yyo + previous_jump_step; yy = yy + jump_step)
				{
					for (xx = xxo; xx < xxo + previous_jump_step; xx = xx + jump_step)
					{
						tmp = gv_Get_Volume(
							&zz_voxel_volume, 
							xx, jump_step, 
							yy, jump_step, 
							zz, jump_step);

						if (!iev_Is_Empty_Volume(&tmp))
						{
							num_Current_Level_Nodes++;

							node.zz_parent_index = k;
							node.zz_parent_zz_children_maks_pose = bit_location;
							node.zz_depth_level = Current_Depth_Level;
							node.zz_start_xyz[0] = xx;
							node.zz_start_xyz[1] = yy;
							node.zz_start_xyz[2] = zz;							
							node.zz_children_mask = 0;

							if (Current_Depth_Level == max_depth)
							{

								node.Is_Root_or_Leaf_or_Not = VCL_LEAF_NODE;
							}
							else
							{
								node.Is_Root_or_Leaf_or_Not = VCL_NOT_LEAF_NODE;
								//node.zz_children_indices.in_Initialize(8);
							}
							//printf("bit_location = %d\n", bit_location);
							zz_nodes.push_back(node);
							zz_nodes[k].zz_children_indices[bit_location] = zz_nodes.size() - 1;
							zz_nodes[k].zz_children_mask = zz_nodes[k].zz_children_mask | zz_powers[bit_location];

							zz_nodes[k].zz_code_string.Insert(bit_location+1, no_one); 
							zz_nodes[k].zz_code_string.Delete(bit_location+2);
						}
						bit_location++;
					}
				}
			}
			//if (!zz_nodes[k].zz_children_indices.z_cne(gnc_Get_Number_Child(zz_nodes[k].zz_children_mask))) return false;
		}
		
		printf("[Set %dth depth] : %d nodes , S index %d, Jump Step %d\n", 
			Current_Depth_Level, 
			num_Current_Level_Nodes, 
			Start_Index_of_Previous_Level_Nodes, 
			jump_step);

		Current_Depth_Level++;
		if (Current_Depth_Level > max_depth) break;
	}
	printf("Vector Size = %d\n", zz_nodes.size());
	//printf("x = %d y = %d z = %d\n", zz_nodes[2].zz_start_xyz[0], zz_nodes[2].zz_start_xyz[1], zz_nodes[2].zz_start_xyz[2]);

	return true;
}
//***********************************************************************************************************
bool VCL_Sparse_Voxel_Octree::gsvo_Generate_Sparse_Voxel_Octree_Top_Down()
//***********************************************************************************************************
{
	VCL_Node node, cpy_node; CKvVolumeBool tmp;
	CKvDepot_of_Voxel depot_voxel;
	CKvSet_of_Voxel set_of_voxel; CKvVoxel *p_voxel;
	CString no_one; CKvStopWatch sw;
	int max_depth, jump_step, previous_jump_step, k, xx, yy, zz, xxo, yyo, zzo;
	int num_Current_Level_Nodes, num_Previous_Level_Nodes, Current_Depth_Level, Start_Index_of_Previous_Level_Nodes;
	int bit_location, num_voxel;


	depot_voxel = zz_docube.gsp_Get_Surface_Voxels();
	depot_voxel.e_Export(&set_of_voxel); depot_voxel.in_Initialize();

	p_voxel = set_of_voxel.vps(num_voxel);
	if (num_voxel == 0) return false;

	max_depth = log2(zz_voxel_resolution);

	//printf("[Generate Sparse Voxel Octree]\n");
	//printf("max depth level = %d\n", max_depth);
	//printf("num voxel = %d\n", num_voxel);
	//printf("[Set 0th depth]\n");

	sw.c_Create(1);
	sw.r_Reset(0);

	node = srn_Set_Root_node(&set_of_voxel);
	zz_nodes.push_back(node);

	Start_Index_of_Previous_Level_Nodes = 0;  no_one = "1";
	num_Current_Level_Nodes = Current_Depth_Level = 1;
	while (1)
	{
		num_Previous_Level_Nodes = num_Current_Level_Nodes;
		Start_Index_of_Previous_Level_Nodes = zz_nodes.size() - num_Previous_Level_Nodes;
		num_Current_Level_Nodes = 0;

		jump_step          = zz_voxel_resolution / pow(2, Current_Depth_Level);
		previous_jump_step = zz_voxel_resolution / pow(2, Current_Depth_Level - 1);

		for (k = Start_Index_of_Previous_Level_Nodes; k < Start_Index_of_Previous_Level_Nodes + num_Previous_Level_Nodes; k++)
		{
			xxo = zz_nodes[k].zz_start_xyz[0];
			yyo = zz_nodes[k].zz_start_xyz[1];
			zzo = zz_nodes[k].zz_start_xyz[2];
			bit_location = 0;
			for (zz = zzo; zz < zzo + previous_jump_step; zz = zz + jump_step)
			{
				for (yy = yyo; yy < yyo + previous_jump_step; yy = yy + jump_step)
				{
					for (xx = xxo; xx < xxo + previous_jump_step; xx = xx + jump_step)
					{

						//if (!iev_Is_Empty_Volume(
						//	zz_nodes[k].zz_set_of_voxel,//CKvMatrixInt *in_set_of_voxels,
						//	xx, jump_step,//int xo, int dx,
						//	yy, jump_step,//int yo, int dy,
						//	zz, jump_step,//int zo, int dz,
						//	node.zz_set_of_voxel))//CKvMatrixInt *out_set_of_voxels))
						if (!iev_Is_Empty_Volume(
							zz_nodes[k].zz_set_of_voxel,//CKvMatrixInt *in_set_of_voxels,
							xx, jump_step,//int xo, int dx,
							yy, jump_step,//int yo, int dy,
							zz, jump_step,//int zo, int dz,
							node.zz_set_of_voxel))//CKvMatrixInt *out_set_of_voxels))
						{
							num_Current_Level_Nodes++;
							
							node.zz_parent_index = k;
							node.zz_parent_zz_children_maks_pose = bit_location;
							node.zz_depth_level = Current_Depth_Level;
							node.zz_start_xyz[0] = xx;
							node.zz_start_xyz[1] = yy;
							node.zz_start_xyz[2] = zz;
							node.zz_children_mask = 0;

							if (Current_Depth_Level == max_depth)
							{
								node.Is_Root_or_Leaf_or_Not = VCL_LEAF_NODE;
							}
							else
							{
								node.Is_Root_or_Leaf_or_Not = VCL_NOT_LEAF_NODE;
							}

							cpy_node.cp_Copy(node);
							zz_nodes.push_back(cpy_node);							
							node.zz_set_of_voxel.clear();

							// Set children information for parent node
							zz_nodes[k].zz_children_indices[bit_location] = zz_nodes.size() - 1;
							zz_nodes[k].zz_children_mask = zz_nodes[k].zz_children_mask | zz_powers[bit_location];

							zz_nodes[k].zz_code_string.Insert(bit_location + 1, no_one);
							zz_nodes[k].zz_code_string.Delete(bit_location + 2);
						}
						bit_location++;
					}
				}
			}			
		}

		printf("[Set %dth depth] : %d nodes , S index %d, Jump Step %d\n",
			Current_Depth_Level,
			num_Current_Level_Nodes,
			Start_Index_of_Previous_Level_Nodes,
			jump_step);

		Current_Depth_Level++;
		if (Current_Depth_Level > max_depth) break;
	}

	printf("Octree generateion: %f\n", sw.get_Get_Elapsed_Time(0));

	//printf("Vector Size = %d\n", zz_nodes.size());
	//printf("x = %d y = %d z = %d\n", zz_nodes[2].zz_start_xyz[0], zz_nodes[2].zz_start_xyz[1], zz_nodes[2].zz_start_xyz[2]);

	return true;
}
//***********************************************************************************************************
bool VCL_Sparse_Voxel_Octree::gsvoc_Generate_Sparse_Voxel_Octree_Top_Down_w_Color()
//***********************************************************************************************************
{
	VCL_Node node, cpy_node; CKvVolumeBool tmp;
	CKvDepot_of_Voxel depot_voxel; CKvSet_of_Voxel set_of_voxel; CKvVoxel *p_voxel;
	CKvDepot_of_RgbaF set_of_color; CKvRgbaF rgbaf;
	CString no_one; CKvStopWatch sw;
	double r, g, b;
	int max_depth, jump_step, previous_jump_step, p, k, xx, yy, zz, xxo, yyo, zzo;
	int num_Current_Level_Nodes, num_Previous_Level_Nodes, Current_Depth_Level, Start_Index_of_Previous_Level_Nodes;
	int bit_location, num_voxel;

	depot_voxel = zz_docube.gsp_Get_Surface_Voxels();
	depot_voxel.e_Export(&set_of_voxel); depot_voxel.in_Initialize();
	set_of_color = zz_docube.gvc_Get_Voxel_Color();

	p_voxel = set_of_voxel.vps(num_voxel);
	if (num_voxel == 0) return false;

	max_depth = log2(zz_voxel_resolution);

	printf("[Generate Sparse Voxel Octree]\n");
	printf("max depth level = %d\n", max_depth);
	printf("num voxel = %d\n", num_voxel);
	printf("[Set 0th depth]\n");

	sw.c_Create(1);
	sw.r_Reset(0);

	node = srn_Set_Root_node(&set_of_voxel, &set_of_color);
	zz_nodes.push_back(node);

	Start_Index_of_Previous_Level_Nodes = 0;  no_one = "1";
	num_Current_Level_Nodes = Current_Depth_Level = 1;
	while (1)
	{
		num_Previous_Level_Nodes = num_Current_Level_Nodes;
		Start_Index_of_Previous_Level_Nodes = zz_nodes.size() - num_Previous_Level_Nodes;
		num_Current_Level_Nodes = 0;

		jump_step = zz_voxel_resolution / pow(2, Current_Depth_Level);
		previous_jump_step = zz_voxel_resolution / pow(2, Current_Depth_Level - 1);

		for (k = Start_Index_of_Previous_Level_Nodes; k < Start_Index_of_Previous_Level_Nodes + num_Previous_Level_Nodes; k++)
		{
			xxo = zz_nodes[k].zz_start_xyz[0];
			yyo = zz_nodes[k].zz_start_xyz[1];
			zzo = zz_nodes[k].zz_start_xyz[2];
			bit_location = 0;
			for (zz = zzo; zz < zzo + previous_jump_step; zz = zz + jump_step)
			{
				for (yy = yyo; yy < yyo + previous_jump_step; yy = yy + jump_step)
				{
					for (xx = xxo; xx < xxo + previous_jump_step; xx = xx + jump_step)
					{
						//if (!iev_Is_Empty_Volume(
						//	zz_nodes[k].zz_set_of_voxel,//std::vector<int> *in_set_of_voxels,
						//	xx, jump_step,//int xo, int dx,
						//	yy, jump_step,//int yo, int dy,
						//	zz, jump_step,//int zo, int dz,
						//	node.zz_set_of_voxel))//std::vector<int> *out_set_of_voxels))
						if (!iev_Is_Empty_Volume(
							zz_nodes[k].zz_set_of_voxel,//std::vector<int> *in_set_of_voxels,
							zz_nodes[k].zz_color_index,//std::vector<int> &in_set_of_color_index,
							xx, jump_step,//int xo, int dx,
							yy, jump_step,//int yo, int dy,
							zz, jump_step,//int zo, int dz,
							node.zz_set_of_voxel,//std::vector<int> &out_set_of_voxels,
							node.zz_color_index))//std::vector<int> &out_set_of_color_index)
						{
							num_Current_Level_Nodes++;

							node.zz_parent_index = k;
							node.zz_parent_zz_children_maks_pose = bit_location;
							node.zz_depth_level = Current_Depth_Level;
							node.zz_start_xyz[0] = xx;
							node.zz_start_xyz[1] = yy;
							node.zz_start_xyz[2] = zz;
							node.zz_children_mask = 0;

							r = g = b = 0.0;
							for (p = 0; p < node.zz_color_index.size(); p++)
							{
								set_of_color.ge_Get_Element(
									node.zz_color_index[p],
									rgbaf);
								r += rgbaf.r; g += rgbaf.g; b += rgbaf.b;
							}
							
							r /= node.zz_color_index.size();
							g /= node.zz_color_index.size();
							b /= node.zz_color_index.size();

							node.zz_red   = (unsigned char)(r*255.0);
							node.zz_green = (unsigned char)(g*255.0);
							node.zz_blue  = (unsigned char)(b*255.0);

							if (Current_Depth_Level == max_depth)
							{
								node.Is_Root_or_Leaf_or_Not = VCL_LEAF_NODE;
							}
							else
							{
								node.Is_Root_or_Leaf_or_Not = VCL_NOT_LEAF_NODE;
							}

							cpy_node.cp_Copy(node);
							zz_nodes.push_back(cpy_node);
							
							node.zz_set_of_voxel.clear();
							node.zz_color_index.clear();

							// Set children information for parent node
							zz_nodes[k].zz_children_indices[bit_location] = zz_nodes.size() - 1;
							zz_nodes[k].zz_children_mask = zz_nodes[k].zz_children_mask | zz_powers[bit_location];

							zz_nodes[k].zz_code_string.Insert(bit_location + 1, no_one);
							zz_nodes[k].zz_code_string.Delete(bit_location + 2);
						}
						bit_location++;
					}
				}
			}
		}

		printf("[Set %dth depth] : %d nodes , S index %d, Jump Step %d\n",
			Current_Depth_Level,
			num_Current_Level_Nodes,
			Start_Index_of_Previous_Level_Nodes,
			jump_step);

		Current_Depth_Level++;
		if (Current_Depth_Level > max_depth) break;
	}

	printf("Octree generateion: %f\n", sw.get_Get_Elapsed_Time(0));

	//printf("Vector Size = %d\n", zz_nodes.size());
	//printf("x = %d y = %d z = %d\n", zz_nodes[2].zz_start_xyz[0], zz_nodes[2].zz_start_xyz[1], zz_nodes[2].zz_start_xyz[2]);

	return true;
}
//***********************************************************************************************************
bool VCL_Sparse_Voxel_Octree::gsvo_Generate_Sparse_Voxel_Octree_Bottom_Up()
//***********************************************************************************************************
{
	return true;
}
//***********************************************************************************************************
VCL_Node VCL_Sparse_Voxel_Octree::srn_Set_Root_node()
//*******************************************************************************************
{
	VCL_Node node;

	node.zz_children_mask = 0;
	node.zz_parent_index = VCL_ROOT_NODE;
	node.zz_children_indices[0] = -1;
	node.zz_children_indices[1] = -1;
	node.zz_children_indices[2] = -1;
	node.zz_children_indices[3] = -1;
	node.zz_children_indices[4] = -1;
	node.zz_children_indices[5] = -1;
	node.zz_children_indices[6] = -1;
	node.zz_children_indices[7] = -1;

	node.Is_Root_or_Leaf_or_Not = VCL_ROOT_NODE;
	node.zz_start_xyz[0] = 0;
	node.zz_start_xyz[1] = 0;
	node.zz_start_xyz[2] = 0;
	node.zz_depth_level = 0;

	return node;
}
//*******************************************************************************************
VCL_Node VCL_Sparse_Voxel_Octree::srn_Set_Root_node(
	CKvSet_of_Voxel *in_set_of_voxel)
//*******************************************************************************************
{
	VCL_Node node;
	CKvVoxel *p_voxel; int num_voxel, k;

	node.zz_children_mask = 0;
	node.zz_parent_index = VCL_ROOT_NODE;
	node.zz_children_indices[0] = -1;
	node.zz_children_indices[1] = -1;
	node.zz_children_indices[2] = -1;
	node.zz_children_indices[3] = -1;
	node.zz_children_indices[4] = -1;
	node.zz_children_indices[5] = -1;
	node.zz_children_indices[6] = -1;
	node.zz_children_indices[7] = -1;

	node.Is_Root_or_Leaf_or_Not = VCL_ROOT_NODE;
	node.zz_start_xyz[0] = 0;
	node.zz_start_xyz[1] = 0;
	node.zz_start_xyz[2] = 0;
	node.zz_depth_level = 0;

	p_voxel = in_set_of_voxel->vps(num_voxel);

	for (k = 0; k < num_voxel; k++)
	{
		node.zz_set_of_voxel.push_back(p_voxel[k].x);
		node.zz_set_of_voxel.push_back(p_voxel[k].y);
		node.zz_set_of_voxel.push_back(p_voxel[k].z);
	}
	
	return node;
}
//*******************************************************************************************
VCL_Node VCL_Sparse_Voxel_Octree::srn_Set_Root_node(
	CKvSet_of_Voxel *in_set_of_voxel, 
	CKvDepot_of_RgbaF *in_set_of_voxel_color)
//*******************************************************************************************
{
	VCL_Node node;
	CKvVoxel *p_voxel; int num_voxel, k;
	CKvRgbaF rgbaf; double red = 0.0, green = 0.0, blue = 0.0;

	node.zz_children_mask = 0;
	node.zz_parent_index = VCL_ROOT_NODE;
	node.zz_children_indices[0] = -1;
	node.zz_children_indices[1] = -1;
	node.zz_children_indices[2] = -1;
	node.zz_children_indices[3] = -1;
	node.zz_children_indices[4] = -1;
	node.zz_children_indices[5] = -1;
	node.zz_children_indices[6] = -1;
	node.zz_children_indices[7] = -1;

	node.Is_Root_or_Leaf_or_Not = VCL_ROOT_NODE;
	node.zz_start_xyz[0] = 0;
	node.zz_start_xyz[1] = 0;
	node.zz_start_xyz[2] = 0;
	node.zz_depth_level = 0;

	p_voxel = in_set_of_voxel->vps(num_voxel);

	for (k = 0; k < num_voxel; k++)
	{



		node.zz_set_of_voxel.push_back(p_voxel[k].x);
		node.zz_set_of_voxel.push_back(p_voxel[k].y);
		node.zz_set_of_voxel.push_back(p_voxel[k].z);
		node.zz_color_index.push_back(k);

		in_set_of_voxel_color->ge_Get_Element(k, rgbaf);
		red += rgbaf.r; green += rgbaf.g; blue += rgbaf.b;
	}

	node.zz_red = (unsigned char)((red / (double)num_voxel)*255.0);
	node.zz_green = (unsigned char)((green / (double)num_voxel) * 255.0);
	node.zz_blue = (unsigned char)((blue / (double)num_voxel) * 255.0);

	return node;
}
//*******************************************************************************************
CKvVolumeBool VCL_Sparse_Voxel_Octree::gv_Get_Volume(
	CKvVolumeBool *in_voxel_volume,
	int xo, int dx, 
	int yo, int dy, 
	int zo, int dz)
//*******************************************************************************************
{
	CKvVolumeBool tmp;
	bool ***p_in, ***p_tmp;
	int xx, yy, zz;

	p_tmp = tmp.c_Create(dz, dy, dx,false);
	p_in     = in_voxel_volume->tp();

	for (zz = zo; zz < zo+dz; zz++)
	{
		for (yy = yo; yy < yo+dy; yy++)
		{
			for (xx = xo; xx < xo+dx; xx++)
			{
				p_tmp[zz - zo][yy - yo][xx - xo] = p_in[zz][yy][xx];
			}
		}
	}

	return tmp;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::iev_Is_Empty_Volume(
	CKvVolumeBool *in_volume)
//*******************************************************************************************
{
	bool *p_v; int k, num;

	p_v=in_volume->vps(num);
	for (k = 0; k < num; k++){ if (p_v[k] == true) { return false; } }
	return true;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::iev_Is_Empty_Volume(
	std::vector<int> &in_set_of_voxels,
	int xo, int dx,
	int yo, int dy,
	int zo, int dz,
	std::vector<int> &out_set_of_voxels)
//*******************************************************************************************
{
	int num, k;

	num = in_set_of_voxels.size();

	for (k = 0; k < num; k+=3)
	{
		if ((xo <= in_set_of_voxels[k]) && (in_set_of_voxels[k] < xo + dx) &&
			(yo <= in_set_of_voxels[k + 1]) && (in_set_of_voxels[k + 1] < yo + dy) &&
			(zo <= in_set_of_voxels[k + 2]) && (in_set_of_voxels[k + 2] < zo + dz))
		{
			out_set_of_voxels.push_back(in_set_of_voxels[k]);
			out_set_of_voxels.push_back(in_set_of_voxels[k+1]);
			out_set_of_voxels.push_back(in_set_of_voxels[k+2]);
		}
	}

	if (out_set_of_voxels.size()>0)
	{
		return false;
	}

	return true;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::iev_Is_Empty_Volume(
	std::vector<int> &in_set_of_voxels,
	std::vector<int> &in_set_of_color_index,
	int xo, int dx,
	int yo, int dy,
	int zo, int dz,
	std::vector<int> &out_set_of_voxels,
	std::vector<int> &out_set_of_color_index)
//*******************************************************************************************
{
	int num, k;

	num = in_set_of_voxels.size();
	for (k = 0; k < num; k += 3)
	{
		if ((xo <= in_set_of_voxels[k]) && (in_set_of_voxels[k] < xo + dx) &&
			(yo <= in_set_of_voxels[k + 1]) && (in_set_of_voxels[k + 1] < yo + dy) &&
			(zo <= in_set_of_voxels[k + 2]) && (in_set_of_voxels[k + 2] < zo + dz))
		{
			out_set_of_voxels.push_back(in_set_of_voxels[k]);
			out_set_of_voxels.push_back(in_set_of_voxels[k + 1]);
			out_set_of_voxels.push_back(in_set_of_voxels[k + 2]);


			out_set_of_color_index.push_back(in_set_of_color_index[(int)(k / 3)]);
		}
	}

	if (out_set_of_voxels.size()>0)
	{
		return false;
	}

	return true;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::iev_Is_Empty_Volume(
	CKvSet_of_Voxel *in_set_of_voxels,
	int xo, int dx,
	int yo, int dy,
	int zo, int dz)
//*******************************************************************************************
{
	CKvVoxel *p_voxel;
	int num, k;

	p_voxel = in_set_of_voxels->vps(num);


	for (k = 0; k < num; k++)
	{
		if ((xo <= p_voxel[k].x) && (p_voxel[k].x < xo + dx) &&
			(yo <= p_voxel[k].y) && (p_voxel[k].y < yo + dy) &&
			(zo <= p_voxel[k].z) && (p_voxel[k].z < zo + dz))
		{
			return false;
		}
	}

	return true;
}
//*******************************************************************************************
void VCL_Sparse_Voxel_Octree::gv_Generate_Volume(
	CKvVolumeBool *out_volume)
//*******************************************************************************************
{
	CKvDepot_of_Voxel depot_voxel;
	CKvSet_of_Voxel set_of_voxel; CKvVoxel *p_voxel;
	int k, num_voxel; bool ***p_volume; 
	int ww, hh, dd;
	
	zz_docube.gsp_Generate_Surface_Voxels(&set_of_voxel);
	depot_voxel = zz_docube.gsp_Get_Surface_Voxels();
	depot_voxel.e_Export(&set_of_voxel); depot_voxel.in_Initialize();

	zz_docube.gr_Get_Resolution(ww, hh, dd);
	p_volume = out_volume->c_Create(dd, hh, ww, false);
	p_voxel  = set_of_voxel.vps(num_voxel);

	for (k = 0; k < num_voxel; k++)
	{
		p_volume[p_voxel[k].z][p_voxel[k].y][p_voxel[k].x] = true; // Error 발생 지점. 
	}
}
//*******************************************************************************************
int VCL_Sparse_Voxel_Octree::gnc_Get_The_Number_of_Child(
	char &in_children_mask)
//*******************************************************************************************
{
	int k, cn;

	cn = 0;
	for (k = 0; k < 8; k++)
	{
		if ((zz_powers[k] & in_children_mask) != 0)
		{
			cn++;
		}
	}

	return cn;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::rbm_Read_Bit_Mask(
	char &in_children_mask,
	int in_bit_index)
//*******************************************************************************************
{
	if (in_bit_index+1 > 8) { return false; }
	if ((zz_powers[in_bit_index] & in_children_mask) != 0){ return true; }
	return false;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::ssvo_Save_Sparse_Voxel_Octree(
	CKvString in_filename,
	int *out_memory_bit_or_NULL)
//*******************************************************************************************
{
	FILE *fp=NULL;
	int memory;
	int sz, k, a, bit_size;

	fopen_s(&fp, (char *)in_filename, "wb");
	if (fp == NULL) return false;
	sz = zz_nodes.size();
	
	memory = 0; 
	bit_size = (int)(log2((double)sz)+0.5);
	for (k = 0; k < sz; k++)
	{
		fwrite(&zz_nodes[k].zz_children_mask, sizeof(char), 1, fp);
		memory += 8;
		for (a = 0; a < 8; a++)
		{
			if (rbm_Read_Bit_Mask(zz_nodes[k].zz_children_mask, a))
			{
				fwrite(&zz_nodes[k].zz_children_indices[a], sizeof(int64_t), 1, fp);
				memory += bit_size;
			}
		}
	}
	printf("SVO Memory = %d\n", memory);
	if (out_memory_bit_or_NULL != NULL){ out_memory_bit_or_NULL = &memory; }
	
	return true;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::ssvo_Save_Sparse_Voxel_Octree_Only_Child_Mask (
	CKvString in_filename,
	int *out_memory_bit_or_NULL)
//*******************************************************************************************
{
	FILE *fp = NULL;
	int memory;
	int sz, k;

	fopen_s(&fp, (char *)in_filename, "wb");
	if (fp == NULL) return false;
	sz = zz_nodes.size();
	memory = 0;
	for (k = 0; k < sz; k++)
	{
		fwrite(&zz_nodes[k].zz_children_mask, sizeof(char), 1, fp);
		memory += 8;

	}
	if (out_memory_bit_or_NULL != NULL){ out_memory_bit_or_NULL = &memory; }
	
	return true;
}
//*******************************************************************************************
bool VCL_Sparse_Voxel_Octree::ssvo_Save_Sparse_Voxel_Octree_String(
	CKvString in_filename, 
	double *out_memory_byte_or_NULL)
//*******************************************************************************************
{
	FILE *fp = NULL;
	int end_idx, max_depth, num_nodes, k, m, sz;
	int shift_factor;
	double memory;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();

	for (k = num_nodes - 1; k > -1; k--)
	{
		if ((zz_nodes[k].zz_depth_level == max_depth-2))
		{
			end_idx = k;
			break;
		}
	}

	for (k = end_idx; k > -1; k--)
	{
		shift_factor = 0;
		for (m = 0; m < 8; m++)
		{
			if (rbm_Read_Bit_Mask(zz_nodes[k].zz_children_mask, m) == true)
			{
				//zz_nodes[zz_nodes[k].zz_children_indices[m]].zz_code_string;
				zz_nodes[k].zz_code_string.Insert(m + 1 + shift_factor, zz_nodes[zz_nodes[k].zz_children_indices[m]].zz_code_string);
				zz_nodes[k].zz_code_string.Delete(m + 1 + 9 + shift_factor);
				shift_factor = shift_factor + (zz_nodes[zz_nodes[k].zz_children_indices[m]].zz_code_string.GetAllocLength() - 1);
			}
		}		
	}

	sz = zz_nodes[0].zz_code_string.GetAllocLength();
	memory = (double)(sz * 2.0) / 8.0;
	if (out_memory_byte_or_NULL != NULL){ out_memory_byte_or_NULL = &memory; }

	CStringA code_stringA(zz_nodes[0].zz_code_string);
	fopen_s(&fp, (char *)in_filename, "w");
	if (fp == NULL) return false;
	fprintf_s(fp, "%s", code_stringA.GetString());

	fclose(fp);
	return true;
}
//*******************************************************************************************
void VCL_Sparse_Voxel_Octree::gtv_Generate_Test_Volume(
	CKvVolumeBool *out_volume)
//*******************************************************************************************
{
	bool ***p;
	int x, y, z, count;

	p=out_volume->c_Create(64, 64, 64, false);
	count = 0;
	for (x = 0; x < 64; x += 2)
	{
		for (y = 0; y < 64; y += 2)
		{
			for (z = 0; z < 64; z += 2)
			{
				p[z][y][x] = true; count++;
			}
		}
	}
}
//*******************************************************************************************
int VCL_Sparse_Voxel_Octree::gms_Get_Memory_size_SVO()
//*******************************************************************************************
{
	int memory;
	int sz, k, a, bit_size;

	sz = zz_nodes.size();

	memory = 0;
	bit_size = (int)(log2((double)sz) + 0.5);
	for (k = 0; k < sz; k++)
	{
		memory += 8;
		for (a = 0; a < 8; a++)
		{
			if (rbm_Read_Bit_Mask(zz_nodes[k].zz_children_mask, a))
			{
				memory += bit_size;
			}
		}
	}

	return memory;
}
//*******************************************************************************************
int VCL_Sparse_Voxel_Octree::gms_Get_Memory_size_SVO_64bit()
//*******************************************************************************************
{
	int memory;
	int sz, k, a;

	sz = zz_nodes.size();

	memory = 0;
	for (k = 0; k < sz; k++)
	{
		memory += 8;
		for (a = 0; a < 8; a++)
		{
			if (rbm_Read_Bit_Mask(zz_nodes[k].zz_children_mask, a))
			{
				memory += 64;
			}
		}
	}

	return memory;
}
//*******************************************************************************************
int VCL_Sparse_Voxel_Octree::gms_Get_Memory_size_Only_Child_Mask()
//*******************************************************************************************
{
	int memory;
	int sz, k;

	sz = zz_nodes.size();
	memory = 0;
	for (k = 0; k < sz; k++)
	{
		memory += 8;
	}

	return memory;
}
//*******************************************************************************************
int VCL_Sparse_Voxel_Octree::gms_Get_Memory_size_String()
//*******************************************************************************************
{
	int end_idx, max_depth, num_nodes, k, m, sz;
	int shift_factor;
	int memory;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();

	for (k = num_nodes - 1; k > -1; k--)
	{
		if ((zz_nodes[k].zz_depth_level == max_depth - 2))
		{
			end_idx = k;
			break;
		}
	}

	for (k = end_idx; k > -1; k--)
	{
		shift_factor = 0;
		for (m = 0; m < 8; m++)
		{
			if (rbm_Read_Bit_Mask(zz_nodes[k].zz_children_mask, m) == true)
			{
				zz_nodes[k].zz_code_string.Insert(m + 1 + shift_factor, zz_nodes[zz_nodes[k].zz_children_indices[m]].zz_code_string);
				zz_nodes[k].zz_code_string.Delete(m + 1 + 9 + shift_factor);
				shift_factor = shift_factor + (zz_nodes[zz_nodes[k].zz_children_indices[m]].zz_code_string.GetAllocLength() - 1);
			}
		}
	}

	sz = zz_nodes[0].zz_code_string.GetAllocLength();
	memory = (sz * 2);


	return memory;
}
//*******************************************************************************************
void VCL_Sparse_Voxel_Octree::gspc_Get_Surface_Points_And_Colors(
	CKvDepot_of_Point3Df *out_surface_points,
	CKvDepot_of_RgbaF *out_set_of_color)
//********************************************************************************************
{
	CKvPoint3D point3d; CKvRgbaF rgbaf;
	int max_depth, num_nodes, num_element, Start_Node_of_MAX_DEPTH;
	int k, sz;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();
	for (k = 0; k < num_nodes; k++)
	{
		if (zz_nodes[k].zz_depth_level == max_depth)
		{
			Start_Node_of_MAX_DEPTH = k;
			break;
		}
	}
	
	num_element = num_nodes - Start_Node_of_MAX_DEPTH;

	out_surface_points->in_Initialize(num_element);
	out_set_of_color->in_Initialize(num_element);
	sz = 0;
	for (k = Start_Node_of_MAX_DEPTH; k < num_nodes; k++)
	{
		point3d.x = (double)zz_nodes[k].zz_start_xyz[0];
		point3d.y = (double)zz_nodes[k].zz_start_xyz[1];
		point3d.z = (double)zz_nodes[k].zz_start_xyz[2];

		rgbaf.r = (float)((float)zz_nodes[k].zz_red / 255.f);
		rgbaf.g = (float)((float)zz_nodes[k].zz_green / 255.f);
		rgbaf.b = (float)((float)zz_nodes[k].zz_blue / 255.f);

		out_surface_points->ap_Append(false, point3d,sz);
		out_set_of_color->ap_Append(false, rgbaf, sz);
	}
}
//********************************************************************************************
void VCL_Sparse_Voxel_Octree::gsv_Get_Surface_Voxel(
	CKvSet_of_Point3D *out_voxels)
//********************************************************************************************
{
	CKvPoint3D *p_point3d;
	int max_depth, num_nodes, num_element, Start_Node_of_MAX_DEPTH;
	int k, count;
	
	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();
	for (k = 0; k < num_nodes; k++)
	{
		if (zz_nodes[k].zz_depth_level == max_depth)
		{
			Start_Node_of_MAX_DEPTH = k;
			break;
		}
	}
	num_element = num_nodes - Start_Node_of_MAX_DEPTH;

	count = 0;
	p_point3d = out_voxels->c_Create(num_element);
	for (k = Start_Node_of_MAX_DEPTH; k < num_nodes; k++)
	{
		p_point3d[count].x = (double)zz_nodes[k].zz_start_xyz[0];
		p_point3d[count].y = (double)zz_nodes[k].zz_start_xyz[1];
		p_point3d[count].z = (double)zz_nodes[k].zz_start_xyz[2];
		count++;
	}
}
//********************************************************************************************
void VCL_Sparse_Voxel_Octree::gsv_Get_Surface_Voxel(
	CKvSet_of_RgbaF *out_voxels_colors)
//********************************************************************************************
{
	CKvRgbaF *p_rgb;
	int max_depth, num_nodes, num_element, Start_Node_of_MAX_DEPTH;
	int k, count;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();
	for (k = 0; k < num_nodes; k++)
	{
		if (zz_nodes[k].zz_depth_level == max_depth)
		{
			Start_Node_of_MAX_DEPTH = k;
			break;
		}
	}
	num_element = num_nodes - Start_Node_of_MAX_DEPTH;
	count = 0;
	p_rgb = out_voxels_colors->c_Create(num_element);
	for (k = Start_Node_of_MAX_DEPTH; k < num_nodes; k++)
	{
		p_rgb[count].r = (float)zz_nodes[k].zz_red/255.f;
		p_rgb[count].g = (float)zz_nodes[k].zz_green/255.f;
		p_rgb[count].b = (float)zz_nodes[k].zz_blue/255.f;
		p_rgb[count].a = 1.f;
		count++;
	}

}
//********************************************************************************************
void VCL_Sparse_Voxel_Octree::gsv_Get_Surface_Voxel_Colors(
	CKvVectorUcharRgb *out_voxels_colors)
//********************************************************************************************
{
	unsigned char *p_r, *p_g, *p_b;
	int max_depth, num_nodes, num_element, Start_Node_of_MAX_DEPTH;
	int k, count;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();
	for (k = 0; k < num_nodes; k++)
	{
		if (zz_nodes[k].zz_depth_level == max_depth)
		{
			Start_Node_of_MAX_DEPTH = k;
			break;
		}
	}
	num_element = num_nodes - Start_Node_of_MAX_DEPTH;
	count = 0;
	out_voxels_colors->c_Create(num_element);
	p_r = out_voxels_colors->vp(p_g, p_b);
	for (k = Start_Node_of_MAX_DEPTH; k < num_nodes; k++)
	{
		p_r[count] = zz_nodes[k].zz_red;
		p_g[count] = zz_nodes[k].zz_green;
		p_b[count] = zz_nodes[k].zz_blue;
		count++;
	}
}
//********************************************************************************************
void VCL_Sparse_Voxel_Octree::gsvl_Get_Surface_Voxel_In_Level(
	CKvSet_of_Point3D *out_voxels, 
	int in_level)
//********************************************************************************************
{
	CKvPoint3D *p_point3d;
	bool flag;
	int max_depth, num_nodes, num_element;
	int k, count, Start_Node_of_MAX_DEPTH, End_Node_of_MAX_DEPTH;

	max_depth = log2(zz_voxel_resolution);
	num_nodes = zz_nodes.size();
	flag = false;
	for (k = 0; k < num_nodes; k++)
	{
		if (zz_nodes[k].zz_depth_level == in_level)
		{
			if (flag == false)
			{
				Start_Node_of_MAX_DEPTH = k;
				flag = true;
			}
			else
			{
				End_Node_of_MAX_DEPTH = k;
			}
		}
		else if (flag == true)
		{
			break;
		}
	}

	//num_element = num_nodes - Start_Node_of_MAX_DEPTH;
	num_element = End_Node_of_MAX_DEPTH - Start_Node_of_MAX_DEPTH + 1;

	count = 0;
	p_point3d = out_voxels->c_Create(num_element);
	for (k = Start_Node_of_MAX_DEPTH; k < End_Node_of_MAX_DEPTH+1; k++)
	{
		p_point3d[count].x = (double)zz_nodes[k].zz_start_xyz[0];
		p_point3d[count].y = (double)zz_nodes[k].zz_start_xyz[1];
		p_point3d[count].z = (double)zz_nodes[k].zz_start_xyz[2];
		count++;
	}

}
//********************************************************************************************
void VCL_Sparse_Voxel_Octree::Get_Level_Given_Block_Size(
	int &in_block_size, 
	int &out_level)
//********************************************************************************************
{
	int max_depth;

	max_depth = log2(zz_voxel_resolution);

	if (in_block_size == 0){ out_level = max_depth; return; }
	if (in_block_size == 2){ out_level = max_depth-1; return; }
	if (in_block_size == 4){ out_level = max_depth-2; return; }
	if (in_block_size == 8){ out_level = max_depth-3; return; }
	if (in_block_size == 16){ out_level = max_depth-4; return; }
	if (in_block_size == 32){ out_level = max_depth-5; return; }
	if (in_block_size == 64){ out_level = max_depth-6; return; }
	if (in_block_size == 128){ out_level = max_depth-7; return; }
	if (in_block_size == 256){ out_level = max_depth-8; return; }
	if (in_block_size == 512){ out_level = max_depth-9; return; }
	if (in_block_size == 1024){ out_level = max_depth-10; return; }
	if (in_block_size == 2048){ out_level = max_depth-11; return; }
	if (in_block_size == 4096){ out_level = max_depth-12; return; }
	if (in_block_size == 8182){ out_level = max_depth-13; return; }
	if (in_block_size == 16384){ out_level = max_depth-14; return; }
	if (in_block_size == 32768){ out_level = max_depth-15; return; }
	if (in_block_size == 65536){ out_level = max_depth-16; return; }

	out_level = -99999;
	return;
}
//********************************************************************************************
