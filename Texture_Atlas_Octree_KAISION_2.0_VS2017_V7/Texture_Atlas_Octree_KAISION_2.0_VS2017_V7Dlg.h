
// Texture_Atlas_Octree_KAISION_2.0_VS2017_V7Dlg.h: 헤더 파일
//

#pragma once


// CTextureAtlasOctreeKAISION20VS2017V7Dlg 대화 상자
class CTextureAtlasOctreeKAISION20VS2017V7Dlg : public CDialogEx
{
// 생성입니다.
public:
	CTextureAtlasOctreeKAISION20VS2017V7Dlg(CWnd* pParent = nullptr);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_TEXTURE_ATLAS_OCTREE_KAISION_20_VS2017_V7_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedDebug();
};
