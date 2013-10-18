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
		blob.dx = blob.xf - blob.xi + 1;
		blob.dy = blob.yf - blob.yi + 1;
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
	blob.correctPixelRatio = float(blob.correctPixelCount)
			/ (float(blob.dx) * float(blob.dy));
}

void BlobDetector::formBlobs(int color) {
	BlobCollection currentBlobs;

	VisionPoint** horizontalPoint = classifier_->horizontalPoint[color];
	uint32_t* horizontalPointCount = classifier_->horizontalPointCount[color];

	uint32_t pointIndex = 0;
	uint32_t pointLine = 0;
	uint32_t pointColumn = 0;

//To do
	uint16_t blobIndex = 0;
	for (uint32_t x = 0; x < horizontalPointCount[0]; ++x) {
		pointIndex = x;

		currentBlobs.push_back(Blob());
		addPointToBlob(horizontalPoint[0][x], pointIndex,
				currentBlobs[blobIndex], blobIndex);
		++blobIndex;

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

				blobIndex = horizontalPoint[y - 1][xx].lbIndex;
				addPointToBlob(horizontalPoint[y][x], pointIndex,
						currentBlobs[blobIndex], blobIndex);
			} else {

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

	// Mark blob as invalid if length lower than 20 or ratio lower than 0.5
	int blobSize = 0;

	for (unsigned int i = 0; i < currentBlobs.size(); ++i) {
		if (currentBlobs[i].dx < 10 || currentBlobs[i].dy < 10
				/*|| currentBlobs[i].correctPixelRatio < 0.5 || currentBlobs[i].getRectRatio() > 1.25*/) {
			currentBlobs[i].invalid = true;
		} else if (currentBlobs[i].invalid == false) {
			++blobSize;
		}
	}

	// Delete Invalid Blob
	horizontalBlob[color].resize(blobSize);

	blobIndex = 0;

	for (unsigned int i = 0; i < currentBlobs.size(); ++i) {
		if (currentBlobs[i].invalid == false) {

			horizontalBlob[color][blobIndex++] = currentBlobs[i];
			for (uint16_t j = 0;
					j < horizontalBlob[color][blobIndex - 1].lpCount; ++j) {

				horizontalPoint[pointLine][pointColumn].lbIndex = blobIndex;

			}
			/*
			std::cout << color << "[ " << blobIndex << " ]: "
					<< horizontalBlob[color][blobIndex - 1].xi << ", "
					<< horizontalBlob[color][blobIndex - 1].yi << ", "
					<< horizontalBlob[color][blobIndex - 1].xf << ", "
					<< horizontalBlob[color][blobIndex - 1].yf << ", "
					<< horizontalBlob[color][blobIndex - 1].dx << ", "
					<< horizontalBlob[color][blobIndex - 1].dy << ", "
					<< horizontalBlob[color][blobIndex - 1].correctPixelCount
					<< ", "
					<< horizontalBlob[color][blobIndex - 1].correctPixelRatio
					<< std::endl;
			*/
		}
	}
}
