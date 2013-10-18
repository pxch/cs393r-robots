#include <vision/BlobDetector.h>
#include <iostream>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) :
		DETECTOR_INITIALIZE, classifier_(classifier) {
	horizontalBlob.resize(NUM_COLORS);
	verticalBlob.resize(NUM_COLORS);
}

bool BlobDetector::isOverlapped(VisionPoint& a, VisionPoint& b) {
	if (a.xf < b.xi || b.xf < a.xi) {
		return false;
	} else {
		return true;
	}
}

void BlobDetector::mergeBlob(BlobCollection& blobs, int color, int indexA,
		int indexB) {
	//merge A into B, mark A as invalid
//	blobs[indexB].lpCount += blobs[indexA].lpCount;
//	for (int i = 0; i < blobs[indexA].lpCount; ++i) {
//		blobs[indexB].lpIndex.push_back(blobs[indexA].lpIndex[i]);
//	}
	for (uint16_t i = 0; i < blobs[indexA].lpCount; ++i) {
		uint32_t pointIndex = blobs[indexA].lpIndex[i];
		addPointToBlob(
				classifier_->horizontalPoint[color][pointIndex >> 16][pointIndex
						& 0xfffful], pointIndex, blobs[indexB], indexB);
	}
	blobs[indexA].invalid = true;
}

void BlobDetector::addPointToBlob(VisionPoint& point, uint32_t pointIndex,
		Blob& blob, uint16_t blobIndex) {
	if (blob.lpCount == 0) {
		blob.xi = point.xi;
		blob.xf = point.xf;
		blob.yi = point.yi;
		blob.yf = point.yf;
		blob.dx = point.dx;
		blob.dy = point.dy;
	} else {
		blob.xi = blob.xi > point.xi ? point.xi : blob.xi;
		blob.yi = blob.yi > point.yi ? point.yi : blob.yi;
		blob.xf = blob.xf > point.xf ? blob.xf : point.xf;
		blob.yf = blob.yf > point.yf ? blob.yf : point.yf;
		blob.dx = blob.xf - blob.xi;
		blob.dy = blob.yf - blob.yi;
	}
	point.lbIndex = blobIndex;
	blob.lpIndex[blob.lpCount++] = pointIndex;
	uint16_t currentPixelCount = blob.correctPixelCount + point.dx;
	if (currentPixelCount > 0) {
		blob.avgX = (blob.avgX * blob.correctPixelCount
				+ (point.xi + point.xf) / 2 * point.dx) / currentPixelCount;
		blob.avgY = (blob.avgY * blob.correctPixelCount + point.yi * point.dx)
				/ currentPixelCount;
	}
	blob.correctPixelCount = currentPixelCount;
}

void BlobDetector::formBlobs(int color) {
	BlobCollection currentBlobs;

	VisionPoint** horizontalPoint = classifier_->horizontalPoint[color];
	uint32_t* horizontalPointCount = classifier_->horizontalPointCount[color];

	uint32_t pointIndex = 0;
	uint32_t pointLine = 0;
	uint32_t pointColumn = 0;
//	Blob* newBlob;

//To do
	uint16_t blobIndex = 0;
//	int currentBlobIndex = 0;
	for (uint32_t x = 0; x < horizontalPointCount[0]; ++x) {
		pointIndex = x;

		currentBlobs.push_back(Blob());
		addPointToBlob(horizontalPoint[0][x], pointIndex,
				currentBlobs[blobIndex], blobIndex);
		++blobIndex;

//			newBlob = new Blob;
//			newBlob->lpCount = 1;
//			newBlob->lpIndex[0] = pointIndex;
//			newBlob->invalid = false;
//			newBlob->xi = 0;
//			newBlob->yi = 0;
//			newBlob->xf = iparams_.width - 1;
//			newBlob->yf = iparams_.height - 1;
//			newBlob->avgX = 0;
//			newBlob->avgY = 0;
//			newBlob->correctPixelRatio = 0.0;
//
//			currentBlobs.push_back(newBlob);
	}
	for (int y = 1; y < iparams_.height; ++y) {
		for (uint32_t x = 0; x < horizontalPointCount[y]; ++x) {
			pointIndex = y << 16 | x;
			uint32_t xx = 0;
			for (xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
				if (isOverlapped(horizontalPoint[y][x],
						horizontalPoint[y - 1][xx])) {
					break;
				}
			}
			if (xx < horizontalPointCount[y - 1]) {
//				currentBlobIndex = horizontalPoint[y - 1][xx].lbIndex;
//				horizontalPoint[y][x].lbIndex = currentBlobIndex;
//				++currentBlobs[currentBlobIndex].lpCount;
//				currentBlobs[currentBlobIndex].lpIndex[currentBlobs[currentBlobIndex].lpCount
//						- 1] = pointIndex;
				blobIndex = horizontalPoint[y - 1][xx].lbIndex;
				addPointToBlob(horizontalPoint[y][x], pointIndex,
						currentBlobs[blobIndex], blobIndex);
			} else {
//				horizontalPoint[y][x].lbIndex = blobCount++;

//				currentBlobs.push_back(
//						Blob(iparams_.width - 1, 0, iparams_.height - 1, 0,
//								pointIndex));

//					newBlob = new Blob;
//					newBlob->lpCount = 1;
//					newBlob->lpIndex[0] = pointIndex;
//					newBlob->invalid = false;
//					newBlob->xi = 0;
//					newBlob->yi = 0;
//					newBlob->xf = iparams_.width - 1;
//					newBlob->yf = iparams_.height - 1;
//					newBlob->avgX = 0;
//					newBlob->avgY = 0;
//					newBlob->correctPixelRatio = 0.0;
//
//					currentBlobs.push_back(newBlob);
				blobIndex = currentBlobs.size();
				currentBlobs.push_back(Blob());
				addPointToBlob(horizontalPoint[y][x], pointIndex,
						currentBlobs[blobIndex], blobIndex);
			}
		}
	}

	// Merge Overlap Blobs
	int indexA, indexB;
	for (int y = 1; y < iparams_.height; ++y) {
		for (uint32_t x = 0; x < horizontalPointCount[y]; ++x) {
			for (uint32_t xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
				indexA = horizontalPoint[y][x].lbIndex;
				indexB = horizontalPoint[y - 1][xx].lbIndex;
				if (isOverlapped(horizontalPoint[y][x],
						horizontalPoint[y - 1][xx]) && indexA != indexB) {
					mergeBlob(currentBlobs, color,
							indexA > indexB ? indexA : indexB,
							indexA > indexB ? indexB : indexA);
				}
			}
		}
	}

	// Mark blob as invalid if area lower than 100
	int blobSize = 0;

	for (unsigned int i = 0; i < currentBlobs.size(); ++i) {
		if (currentBlobs[i].correctPixelCount < 100) {
			currentBlobs[i].invalid = true;
		} else if (currentBlobs[i].invalid == false) {
			++blobSize;
		}
	}

	// Delete Invalid Blob
	horizontalBlob[color].resize(blobSize);

	blobIndex = 0;
	uint16_t xi_1, xi_2, xf_1, xf_2, yi_1, yi_2, yf_1, yf_2, dx_2, dy_2;
	for (unsigned int i = 0; i < currentBlobs.size(); ++i) {
		if (currentBlobs[i].invalid == false) {
//				Blob validBlob = currentBlobs[i];
//				horizontalBlob[c].push_back(Blob(currentBlobs[i]));
//				++blobCount;
			horizontalBlob[color][blobIndex++] = currentBlobs[i];
			for (uint16_t j = 0;
					j < horizontalBlob[color][blobIndex - 1].lpCount; ++j) {
//				pointIndex = horizontalBlob[color][blobCount - 1].lpIndex[j];
//				pointLine = pointIndex >> 16;
//				pointColumn = pointIndex & 0xfffful;

				horizontalPoint[pointLine][pointColumn].lbIndex = blobIndex;

//				xi_1 = horizontalBlob[color][blobCount - 1].xi;
//				xf_1 = horizontalBlob[color][blobCount - 1].xf;
//				yi_1 = horizontalBlob[color][blobCount - 1].yi;
//				yf_1 = horizontalBlob[color][blobCount - 1].yf;
//
//				xi_2 = horizontalPoint[pointLine][pointColumn].xi;
//				xf_2 = horizontalPoint[pointLine][pointColumn].xf;
//				yi_2 = horizontalPoint[pointLine][pointColumn].yi;
//				yf_2 = horizontalPoint[pointLine][pointColumn].yf;
//
//				dx_2 = horizontalPoint[pointLine][pointColumn].dx;
//
//				horizontalBlob[color][blobCount - 1].xi =
//						xi_1 > xi_2 ? xi_2 : xi_1;
//				horizontalBlob[color][blobCount - 1].yi =
//						yi_1 > yi_2 ? yi_2 : yi_1;
//
//				horizontalBlob[color][blobCount - 1].xf =
//						xf_1 > xf_2 ? xf_1 : xf_2;
//				horizontalBlob[color][blobCount - 1].yf =
//						yf_1 > yf_2 ? yf_1 : yf_2;
//
//				horizontalBlob[color][blobCount - 1].avgX += (xi_2 + xf_2) / 2
//						* dx_2;
//
//				horizontalBlob[color][blobCount - 1].avgY += yi_2 * dx_2;
//
//				horizontalBlob[color][blobCount - 1].correctPixelCount += dx_2;
			}
//			if (horizontalBlob[color][blobCount - 1].correctPixelCount > 0) {
//				horizontalBlob[color][blobCount - 1].avgX /=
//						horizontalBlob[color][blobCount - 1].correctPixelCount;
//				horizontalBlob[color][blobCount - 1].avgY /=
//						horizontalBlob[color][blobCount - 1].correctPixelCount;
//			}
//			horizontalBlob[color][blobCount - 1].dx =
//					horizontalBlob[color][blobCount - 1].xf
//							- horizontalBlob[color][blobCount - 1].xi + 1;
//			horizontalBlob[color][blobCount - 1].dy =
//					horizontalBlob[color][blobCount - 1].yf
//							- horizontalBlob[color][blobCount - 1].yi + 1;
			std::cout << color << "[ " << blobIndex << " ]: "
					<< horizontalBlob[color][blobIndex - 1].xi << ", "
					<< horizontalBlob[color][blobIndex - 1].yi << ", "
					<< horizontalBlob[color][blobIndex - 1].xf << ", "
					<< horizontalBlob[color][blobIndex - 1].yf << std::endl;
		}
	}
}
