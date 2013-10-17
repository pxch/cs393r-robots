#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) :
		DETECTOR_INITIALIZE, classifier_(classifier) {
	horizontalBlob.resize(NUM_COLORS);
	verticalBlob.resize(NUM_COLORS);
}

bool BlobDetector::IsOverlapped(VisionPoint& a, VisionPoint& b) {
	if (a.xf < b.xi || b.xf < a.xi) {
		return false;
	} else {
		return true;
	}
}

void BlobDetector::MergeBlob(BlobCollection& blobs, int indexA, int indexB) {
	//merge A into B, mark A as invalid
	blobs[indexB].lpCount += blobs[indexA].lpCount;
	for (int i = 0; i < blobs[indexA].lpCount; ++i) {
		blobs[indexB].lpIndex.push_back(blobs[indexA].lpIndex[i]);
	}
	blobs[indexA].invalid = true;
}

void BlobDetector::detectBlob() {
	for (int c = 0; c < NUM_COLORS; c++) {
		BlobCollection currentBlobs;

		VisionPoint** horizontalPoint = classifier_->horizontalPoint[c];
		uint32_t* horizontalPointCount = classifier_->horizontalPointCount[c];

		uint32_t pointIndex = 0;
		uint32_t pointLine = 0;
		uint32_t pointColumn = 0;
		Blob* newBlob;

		//To do
		int blobCount = 0;
		int currentBlobIndex = 0;
		for (uint32_t x = 0; x < horizontalPointCount[0]; ++x) {
			pointIndex = x;
			horizontalPoint[0][x].lbIndex = blobCount++;
			newBlob = new Blob;
			newBlob->lpCount = 1;
			newBlob->lpIndex[0] = pointIndex;
			newBlob->invalid = false;
			newBlob->xi = 0;
			newBlob->yi = 0;
			newBlob->xf = iparams_.width - 1;
			newBlob->yf = iparams_.height - 1;
			newBlob->avgX = 0;
			newBlob->avgY = 0;
			newBlob->correctPixelRatio = 0.0;

			currentBlobs.push_back(*newBlob);
		}
		for (int y = 1; y < iparams_.height; ++y) {
			for (uint32_t x = 0; x < horizontalPointCount[y]; ++x) {
				pointIndex = y << 16 | x;
				uint32_t xx = 0;
				for (xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
					if (IsOverlapped(horizontalPoint[y][x],
							horizontalPoint[y - 1][xx])) {
						break;
					}
				}
				if (xx < horizontalPointCount[y - 1]) {
					currentBlobIndex = horizontalPoint[y - 1][xx].lbIndex;
					horizontalPoint[y][x].lbIndex = currentBlobIndex;
					++currentBlobs[currentBlobIndex].lpCount;
					currentBlobs[currentBlobIndex].lpIndex[currentBlobs[currentBlobIndex].lpCount
							- 1] = pointIndex;
				} else {
					horizontalPoint[y][x].lbIndex = blobCount++;
					newBlob = new Blob;
					newBlob->lpCount = 1;
					newBlob->lpIndex[0] = pointIndex;
					newBlob->invalid = false;
					newBlob->xi = 0;
					newBlob->yi = 0;
					newBlob->xf = iparams_.width - 1;
					newBlob->yf = iparams_.height - 1;
					newBlob->avgX = 0;
					newBlob->avgY = 0;
					newBlob->correctPixelRatio = 0.0;

					currentBlobs.push_back(*newBlob);
				}
			}
		}

		// Merge Overlap Blobs
//		int indexA, indexB;
//		for (int y = 1; y < iparams_.height; ++y) {
//			for (uint32_t x = 0; x < horizontalPointCount[y]; ++x) {
//				for (uint32_t xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
//					indexA = horizontalPoint[y][x].lbIndex;
//					indexB = horizontalPoint[y - 1][xx].lbIndex;
//					if (IsOverlapped(horizontalPoint[y][x],
//							horizontalPoint[y - 1][xx]) && indexA != indexB) {
//						MergeBlob(currentBlobs,
//								indexA > indexB ? indexA : indexB,
//								indexA > indexB ? indexB : indexA);
//					}
//				}
//			}
//		}

		// Delete Invalid Blob
//		blobCount = 0;
//		uint16_t xi_1, xi_2, xf_1, xf_2, yi_1, yi_2, yf_1, yf_2, dx_2, dy_2;
//		for (unsigned int i = 0; i < currentBlobs.size(); ++i) {
//			if (currentBlobs[i].invalid == false) {
//				horizontalBlob[c].push_back(currentBlobs[i]);
//				++blobCount;
//				for (uint16_t j = 0;
//						j < horizontalBlob[c][blobCount - 1].lpCount; ++j) {
//					pointIndex = horizontalBlob[c][blobCount - 1].lpIndex[j];
//					pointLine = pointIndex >> 16;
//					pointColumn = pointIndex - pointLine;
//					horizontalPoint[pointLine][pointColumn].lbIndex = blobCount;
//
//					xi_1 = horizontalBlob[c][blobCount - 1].xi;
//					xf_1 = horizontalBlob[c][blobCount - 1].xf;
//					yi_1 = horizontalBlob[c][blobCount - 1].yi;
//					yf_1 = horizontalBlob[c][blobCount - 1].yf;
//
//					xi_2 = horizontalPoint[pointLine][pointColumn].xi;
//					xf_2 = horizontalPoint[pointLine][pointColumn].xf;
//					yi_2 = horizontalPoint[pointLine][pointColumn].yi;
//					yf_2 = horizontalPoint[pointLine][pointColumn].yf;
//
//					dx_2 = horizontalPoint[pointLine][pointColumn].dx;
//					dy_2 = horizontalPoint[pointLine][pointColumn].dy;
//
//					horizontalBlob[c][blobCount - 1].xi = xi_1 > xi_2 ? xi_2 : xi_1;
//					horizontalBlob[c][blobCount - 1].yi = yi_1 > yi_2 ? yi_2 : yi_1;
//
//					horizontalBlob[c][blobCount - 1].xf = xf_1 > xf_2 ? xf_1 : xf_2;
//					horizontalBlob[c][blobCount - 1].yf = yf_1 > yf_2 ? yf_1 : yf_2;
//
//					horizontalBlob[c][blobCount - 1].avgX += (xi_2 + xf_2) / 2 * dx_2;
//
//					horizontalBlob[c][blobCount - 1].avgY += yi_2 * dx_2;
//
//					horizontalBlob[c][blobCount - 1].correctPixelRatio += dx_2;
//				}
//				horizontalBlob[c][blobCount - 1].avgX /=
//						horizontalBlob[c][blobCount - 1].correctPixelRatio;
//				horizontalBlob[c][blobCount - 1].avgY /=
//						horizontalBlob[c][blobCount - 1].correctPixelRatio;
//			}
//		}
	}
}
