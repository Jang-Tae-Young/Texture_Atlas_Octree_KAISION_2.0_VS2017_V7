
// Texture_Atlas_Octree_KAISION_2.0_VS2017_V7.h: PROJECT_NAME 응용 프로그램에 대한 주 헤더 파일입니다.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH에 대해 이 파일을 포함하기 전에 'stdafx.h'를 포함합니다."
#endif

#include "resource.h"		// 주 기호입니다.


// CTextureAtlasOctreeKAISION20VS2017V7App:
// 이 클래스의 구현에 대해서는 Texture_Atlas_Octree_KAISION_2.0_VS2017_V7.cpp을(를) 참조하세요.
//

class CTextureAtlasOctreeKAISION20VS2017V7App : public CWinApp
{
public:
	CTextureAtlasOctreeKAISION20VS2017V7App();

// 재정의입니다.
public:
	virtual BOOL InitInstance();

// 구현입니다.

	DECLARE_MESSAGE_MAP()
};

extern CTextureAtlasOctreeKAISION20VS2017V7App theApp;
