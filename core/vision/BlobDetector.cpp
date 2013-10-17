#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) :
		DETECTOR_INITIALIZE, classifier_(classifier) {
	horizontalBlob.resize(NUM_COLORS);
	verticalBlob.resize(NUM_COLORS);
}

bool BlobDetector::IsOverLapped(VisionPoint& a, VisionPoint& b) {
	if (a.xf < b.xi || b.xf < a.xi) {
		return false;
	} else {
		return true;
	}
}

void BlobDetector::MergeBlob(int color, int indexA, int indexB) {

}

void BlobDetector::detectBlob() {
	for (int c = 0; c < NUM_COLORS; c++) {
		VisionPoint** horizontalPoint = classifier_->horizontalPoint[c];
		uint32_t* horizontalPointCount = classifier_->horizontalPointCount[c];

		uint32_t pointIndex = 0;
		Blob* newBlob;

		//To do
		int blobCount = 0;
		int currentBlob = 0;
		for (int x = 0; x < horizontalPointCount[0]; ++x) {
			pointIndex = x;
			horizontalPoint[0][x].lbIndex = blobCount++;
			newBlob = new Blob;
			newBlob->lpCount = 1;
			newBlob->lpIndex.push_back(pointIndex);
			horizontalBlob[c].push_back(*newBlob);
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
				if (xx < horizontalPointCount[y - 1]) {
					currentBlob = horizontalPoint[y - 1][xx].lbIndex;
					horizontalPoint[y][x].lbIndex = currentBlob;
					++horizontalBlob[c][currentBlob].lpCount;
					horizontalBlob[c][currentBlob].lpIndex.push_back(
							pointIndex);
				} else {
					horizontalPoint[y][x].lbIndex = blobCount++;
					newBlob = new Blob;
					newBlob->lpCount = 1;
					newBlob->lpIndex.push_back(pointIndex);
					horizontalBlob[c].push_back(*newBlob);
				}
			}
		}

		// Merge Overlap Blobs
		for (int y = 1; y < iparams_.height; ++y) {
			for (int x = 0; x < horizontalPointCount[y]; ++x) {
				for (int xx = 0; xx < horizontalPointCount[y - 1]; ++xx) {
					if (IsOverlapped(horizontalPoint[y][x],
							horizontalPoint[y - 1][xx])
							&& horizontalPoint[y][x].lbIndex
									!= horizontalPoint[y - 1][xx].lbIndex) {
						MergeBlob(c, horizontalPoint[y][x].lbIndex, horizontalPoint[y - 1][xx].lbIndex);
					}
				}
			}
		}
	}
}
