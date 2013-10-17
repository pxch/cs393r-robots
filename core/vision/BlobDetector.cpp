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
		int pointLine = 0;
		int pointColumn = 0;
		Blob* newBlob;

		//To do
		int blobCount = 0;
		int currentBlobIndex = 0;
		for (int x = 0; x < horizontalPointCount[0]; ++x) {
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
			for (int x = 0; x < horizontalPointCount[y]; ++x) {
				pointIndex = y << 16 | x;
				int xx = 0;
				for (xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
					if (IsOverlapped(horizontalPoint[y][x],
							horizontalPoint[y - 1][xx])) {
						break;
					}
				}
				if (xx < horizontalPointCount[y -horizontalBlob[c] 1]) {
					currentBlobIndex = horizontalPoint[y - 1][xx].lbIndex;
					horizontalPoint[y][x].lbIndex = currentBlobIndex;
					++currentBlobs[currentBlobIndex].lpCount;
					currentBlobs[currentBlobIndex].lpIndex[currentBlobs[currentBlobIndex].lpCount - 1] = pointIndex;
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
		int indexA, indexB;
		for (int y = 1; y < iparams_.height; ++y) {
			for (int x = 0; x < horizontalPointCount[y]; ++x) {
				for (int xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
					indexA = horizontalPoint[y][x].lbIndex;
					indexB = horizontalPoint[y-1][xx].lbIndex;
					if (IsOverlapped(horizontalPoint[y][x],
							horizontalPoint[y - 1][xx])
							&& indexA != indexB) {
						MergeBlob(currentBlobs, indexA > indexB? indexA : indexB, indexA > indexB? indexB : indexA);
					}
				}
			}
		}

		// Delete Invalid Blob
		blobCount = 0;
		Blob* curBlob = NULL;
		VisionPoint* curPoint = NULL;
		for (int i = 0; i < currentBlobs.size(); ++i) {
			if (currentBlobs[i].invalid == false) {
				horizontalBlob[c].push_back(currentBlobs[i]);
				++blobCount;
				curBlob = horizontalBlob[c][blobCount - 1];
				for (uint16_t j = 0; j < curBlob->lpCount; ++ j) {
					pointIndex = curBlob->lpIndex[j];
					pointLine = pointIndex >> 16;
					pointColumn = pointIndex & 0x0000ffff;
					curPoint = horizontalPoint[c][pointLine][pointColumn];
					curPoint->lbIndex = blobCount;
					curBlob->xi = curBlob->xi > curPoint->xi ? curPoint->xi : curBlob->xi;
					curBlob->yi = curBlob->yi > curPoint->yi ? curPoint->yi : curBlob->yi;
					curBlob->xf = curBlob->xf > curPoint->xf ? curBlob->xf : curPoint->xf;
					curBlob->yf = curBlob->yf > curPoint->yf ? curBlob->yf : curPoint->yf;
					curBlob->avgX += (curPoint->xi + curPoint->xf) / 2 * curPoint->dx;
					curBlob->avgY += curPoint->yi * curPoint->dy;
					curBlob->correctPixelRatio += curPoint->dx;
				}
				curBlob->avgX /= curBlob->correctPixelRatio;
				curBlob->avgY /= curBlob->correctPixelRatio;
			}
		}
	}
}
