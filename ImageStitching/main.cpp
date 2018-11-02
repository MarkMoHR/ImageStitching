#include "stdafx.h"
#include "MyMatching.h"
#include "MyBlending.h"

int main() {
	char* inputAddr1 = "Input/1.bmp";
	char* inputAddr2 = "Input/2.bmp";

	MySift mySift1(inputAddr1, 1);
	mySift1.SiftMainProcess();
	mySift1.saveImgWithKeypoint("Output/1-2/1_kp.bmp");

	MySift mySift2(inputAddr2, 1);
	mySift2.SiftMainProcess();
	mySift2.saveImgWithKeypoint("Output/1-2/2_kp.bmp");

	MyMatching myMatching(mySift1.getKeyPointsCount(), mySift1.getFirstKeyDescriptors(),
		mySift2.getKeyPointsCount(), mySift2.getFirstKeyDescriptors());
	myMatching.featureMatchMainProcess();
	myMatching.drawOriKeypointOnImg(inputAddr1, inputAddr2, "Output/1-2/1_kp_real.bmp", "Output/1-2/2_kp_real.bmp");
	myMatching.mixImageAndDrawPairLine("Output/1-2/mixImg.bmp", "Output/1-2/mixImgWithLine.bmp");
	myMatching.myRANSACtoFindKpTransAndDrawOut("Output/1-2/mixImgWithLine_fixed.bmp");

	MyBlending myBlending(myMatching.getMatchVec().col, myMatching.getMatchVec().row);
	myBlending.blendingMainProcess(inputAddr1, inputAddr2);
	myBlending.saveBlendedImg("Output/1-2/blendedImg.bmp");

	return 0;
}

