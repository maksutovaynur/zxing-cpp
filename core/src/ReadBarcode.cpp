/*
* Copyright 2019 Axel Waggershauser
*/
// SPDX-License-Identifier: Apache-2.0

#include "ReadBarcode.h"

#if !defined(ZXING_READERS) && !defined(ZXING_WRITERS)
#include "Version.h"
#endif

#ifdef ZXING_READERS
#include "GlobalHistogramBinarizer.h"
#include "HybridBinarizer.h"
#include "MultiFormatReader.h"
#include "Pattern.h"
#include "ThresholdBinarizer.h"
#endif

#include <algorithm>
#include <climits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ZXing {

#ifdef ZXING_READERS

class LumImage : public Image
{
public:
	using Image::Image;

	uint8_t* data() { return const_cast<uint8_t*>(Image::data()); }
};

template<typename P>
static LumImage ExtractLum(const ImageView& iv, P projection)
{
	LumImage res(iv.width(), iv.height());

	auto* dst = res.data();
	for(int y = 0; y < iv.height(); ++y)
		for(int x = 0, w = iv.width(); x < w; ++x)
			*dst++ = projection(iv.data(x, y));

	return res;
}

class LumImagePyramid
{
	std::vector<LumImage> buffers;

	template<int N>
	void addLayer()
	{
		auto siv = layers.back();
		buffers.emplace_back(siv.width() / N, siv.height() / N);
		layers.push_back(buffers.back());
		auto& div = buffers.back();
		auto* d   = div.data();

		for (int dy = 0; dy < div.height(); ++dy)
			for (int dx = 0; dx < div.width(); ++dx) {
				int sum = (N * N) / 2;
				for (int ty = 0; ty < N; ++ty)
					for (int tx = 0; tx < N; ++tx)
						sum += *siv.data(dx * N + tx, dy * N + ty);
				*d++ = sum / (N * N);
			}
	}

	void addLayer(int factor)
	{
		// help the compiler's auto-vectorizer by hard-coding the scale factor
		switch (factor) {
		case 2: addLayer<2>(); break;
		case 3: addLayer<3>(); break;
		case 4: addLayer<4>(); break;
		default: throw std::invalid_argument("Invalid ReaderOptions::downscaleFactor"); break;
		}
	}

public:
	std::vector<ImageView> layers;

	LumImagePyramid(const ImageView& iv, int threshold, int factor)
	{
		if (factor < 2)
			throw std::invalid_argument("Invalid ReaderOptions::downscaleFactor");

		layers.push_back(iv);
		// TODO: if only matrix codes were considered, then using std::min would be sufficient (see #425)
		while (threshold > 0 && std::max(layers.back().width(), layers.back().height()) > threshold &&
			   std::min(layers.back().width(), layers.back().height()) >= factor)
			addLayer(factor);
#if 0
		// Reversing the layers means we'd start with the smallest. that can make sense if we are only looking for a
		// single symbol. If we start with the higher resolution, we get better (high res) position information.
		// TODO: see if masking out higher res layers based on found symbols in lower res helps overall performance.
		std::reverse(layers.begin(), layers.end());
#endif
	}
};

// Simple rectangle structure for QR code region detection
struct SimpleRect {
	int x, y, width, height;

	bool contains(int px, int py) const {
		return px >= x && px < x + width && py >= y && py < y + height;
	}
};

// Debug information structure for diagnostics
struct DebugInfo {
	std::vector<SimpleRect> detectedRegions;
	int regionsProcessed = 0;
	bool usedFallback = false;
	bool usedFloodFill = false;
	std::vector<std::vector<uint8_t>> floodFillImages; // Store flood fill images as raw luminance data
	int floodFillWidth = 0;
	int floodFillHeight = 0;
};

// Helper to translate position by offset
static Position Translate(const Position& pos, int offsetX, int offsetY)
{
	Position result = pos;
	for (auto& p : result) {
		p.x += offsetX;
		p.y += offsetY;
	}
	return result;
}

// Helper to extract a region from luminance image (returns via ImageView)
static ImageView ExtractRegion(const ImageView& iv, const SimpleRect& region, LumImage& storage)
{
	storage = LumImage(region.width, region.height);
	for (int y = 0; y < region.height; ++y) {
		for (int x = 0; x < region.width; ++x) {
			int srcX = region.x + x;
			int srcY = region.y + y;
			if (srcX >= 0 && srcX < iv.width() && srcY >= 0 && srcY < iv.height()) {
				auto* src = iv.data(srcX, srcY);
				uint8_t lum;
				if (iv.format() == ImageFormat::Lum) {
					lum = *src;
				} else if (iv.format() == ImageFormat::RGB || iv.format() == ImageFormat::RGBA) {
					lum = RGBToLum(src[0], src[1], src[2]);
				} else if (iv.format() == ImageFormat::BGR || iv.format() == ImageFormat::BGRA) {
					lum = RGBToLum(src[2], src[1], src[0]);
				} else {
					lum = RGBToLum(src[RedIndex(iv.format())], src[GreenIndex(iv.format())], src[BlueIndex(iv.format())]);
				}
				storage.data()[y * region.width + x] = lum;
			}
		}
	}
	return storage;
}

// Detect high-contrast regions that likely contain QR codes
// For black backgrounds with white QR codes, look for brighter regions
static std::vector<SimpleRect> DetectQRCodeRegions(const ImageView& iv)
{
	std::vector<SimpleRect> regions;
	const int width = iv.width();
	const int height = iv.height();
	const int blockSize = 15; // Larger blocks to better capture QR patterns

	const int blocksX = (width + blockSize - 1) / blockSize;
	const int blocksY = (height + blockSize - 1) / blockSize;

	// Calculate overall image mean for comparison
	long long totalSum = 0;
	int totalCount = 0;
	for (int y = 0; y < height; y += 2) { // Sample every other pixel for speed
		for (int x = 0; x < width; x += 2) {
			auto* src = iv.data(x, y);
			uint8_t lum;
			if (iv.format() == ImageFormat::Lum) {
				lum = *src;
			} else if (iv.format() == ImageFormat::RGB || iv.format() == ImageFormat::RGBA) {
				lum = RGBToLum(src[0], src[1], src[2]);
			} else if (iv.format() == ImageFormat::BGR || iv.format() == ImageFormat::BGRA) {
				lum = RGBToLum(src[2], src[1], src[0]);
			} else {
				lum = RGBToLum(src[RedIndex(iv.format())], src[GreenIndex(iv.format())], src[BlueIndex(iv.format())]);
			}
			totalSum += lum;
			totalCount++;
		}
	}
	int imageMean = totalCount > 0 ? totalSum / totalCount : 128;

	// Store block means
	std::vector<int> blockMeans(blocksX * blocksY, 0);
	std::vector<bool> interestingBlocks(blocksX * blocksY, false);

	// Calculate mean for each block
	for (int by = 0; by < blocksY; ++by) {
		for (int bx = 0; bx < blocksX; ++bx) {
			int sum = 0;
			int count = 0;
			int minLum = 255;
			int maxLum = 0;

			int startY = by * blockSize;
			int startX = bx * blockSize;
			int endY = std::min(startY + blockSize, height);
			int endX = std::min(startX + blockSize, width);

			for (int y = startY; y < endY; ++y) {
				for (int x = startX; x < endX; ++x) {
					auto* src = iv.data(x, y);
					uint8_t lum;
					if (iv.format() == ImageFormat::Lum) {
						lum = *src;
					} else if (iv.format() == ImageFormat::RGB || iv.format() == ImageFormat::RGBA) {
						lum = RGBToLum(src[0], src[1], src[2]);
					} else if (iv.format() == ImageFormat::BGR || iv.format() == ImageFormat::BGRA) {
						lum = RGBToLum(src[2], src[1], src[0]);
					} else {
						lum = RGBToLum(src[RedIndex(iv.format())], src[GreenIndex(iv.format())], src[BlueIndex(iv.format())]);
					}
					sum += lum;
					minLum = std::min(minLum, (int)lum);
					maxLum = std::max(maxLum, (int)lum);
					count++;
				}
			}

			if (count > 0) {
				int blockIdx = by * blocksX + bx;
				int mean = sum / count;
				int range = maxLum - minLum;
				blockMeans[blockIdx] = mean;

				// Mark blocks that are significantly different from image mean
				// OR have significant internal range (QR pattern variation)
				// For black backgrounds: look for brighter blocks (mean > imageMean)
				// For white backgrounds: look for darker blocks (mean < imageMean)
				int meanDiff = std::abs(mean - imageMean);

				if (meanDiff > 20 || range > 30) {
					interestingBlocks[blockIdx] = true;
				}
			}
		}
	}

	// Also mark blocks adjacent to interesting blocks with sufficient contrast
	std::vector<bool> expandedBlocks = interestingBlocks;
	for (int by = 0; by < blocksY; ++by) {
		for (int bx = 0; bx < blocksX; ++bx) {
			int blockIdx = by * blocksX + bx;
			if (!interestingBlocks[blockIdx]) continue;

			// Check 8 neighbors
			for (int dy = -1; dy <= 1; ++dy) {
				for (int dx = -1; dx <= 1; ++dx) {
					if (dx == 0 && dy == 0) continue;
					int ny = by + dy;
					int nx = bx + dx;
					if (ny >= 0 && ny < blocksY && nx >= 0 && nx < blocksX) {
						int nidx = ny * blocksX + nx;
						if (std::abs(blockMeans[blockIdx] - blockMeans[nidx]) > 15) {
							expandedBlocks[nidx] = true;
						}
					}
				}
			}
		}
	}

	// Find connected components of interesting blocks and create bounding boxes
	std::vector<bool> visited(blocksX * blocksY, false);
	for (int by = 0; by < blocksY; by++) {
		for (int bx = 0; bx < blocksX; bx++) {
			int blockIdx = by * blocksX + bx;
			if (expandedBlocks[blockIdx] && !visited[blockIdx]) {
				// Found a new region - find its bounds
				int minX = bx, maxX = bx;
				int minY = by, maxY = by;

				// Simple flood fill to find connected high-contrast blocks
				std::vector<std::pair<int, int>> stack;
				stack.push_back({bx, by});
				visited[blockIdx] = true;

				while (!stack.empty()) {
					auto [cx, cy] = stack.back();
					stack.pop_back();

					minX = std::min(minX, cx);
					maxX = std::max(maxX, cx);
					minY = std::min(minY, cy);
					maxY = std::max(maxY, cy);

					// Check 4-connected neighbors
					for (auto [dx, dy] : {std::pair{-1,0}, {1,0}, {0,-1}, {0,1}}) {
						int nx = cx + dx;
						int ny = cy + dy;
						if (nx >= 0 && nx < blocksX && ny >= 0 && ny < blocksY) {
							int nidx = ny * blocksX + nx;
							if (expandedBlocks[nidx] && !visited[nidx]) {
								visited[nidx] = true;
								stack.push_back({nx, ny});
							}
						}
					}
				}

				// Create bounding rectangle with generous padding
				// QR codes need extra space around finder patterns
				const int padding = blockSize * 6; // Very generous padding
				SimpleRect rect;
				rect.x = std::max(0, minX * blockSize - padding);
				rect.y = std::max(0, minY * blockSize - padding);
				rect.width = std::min(width - rect.x, (maxX - minX + 1) * blockSize + 2 * padding);
				rect.height = std::min(height - rect.y, (maxY - minY + 1) * blockSize + 2 * padding);

				// Only add reasonably sized regions (likely to be QR codes)
				// Minimum: at least 2x2 blocks (30x30 pixels with blockSize=15)
				// Maximum: accept up to 95% of image
				if (rect.width >= blockSize * 2 && rect.height >= blockSize * 2 &&
				    rect.width <= width * 0.95 && rect.height <= height * 0.95) {
					regions.push_back(rect);
				}
			}
		}
	}

	// Calculate high-variance area for each region and sort by area (highest first)
	// QR codes have large areas with high local variance, while text has concentrated variance
	struct RegionWithScore {
		SimpleRect rect;
		int highVarianceArea; // Number of pixels in high-variance blocks
	};
	std::vector<RegionWithScore> rankedRegions;

	for (const auto& rect : regions) {
		// Scan region in small blocks and count blocks with high variance
		const int scanBlockSize = 5; // Small blocks to detect local variance
		const int varianceThreshold = 100; // Threshold for "high variance"
		int highVariancePixels = 0;

		for (int by = rect.y; by < rect.y + rect.height; by += scanBlockSize) {
			for (int bx = rect.x; bx < rect.x + rect.width; bx += scanBlockSize) {
				int sum = 0;
				int sumSq = 0;
				int count = 0;
				int minLum = 255;
				int maxLum = 0;

				// Calculate variance for this small block
				int endY = std::min(by + scanBlockSize, std::min(rect.y + rect.height, height));
				int endX = std::min(bx + scanBlockSize, std::min(rect.x + rect.width, width));

				for (int y = by; y < endY; ++y) {
					for (int x = bx; x < endX; ++x) {
						auto* src = iv.data(x, y);
						uint8_t lum;
						if (iv.format() == ImageFormat::Lum) {
							lum = *src;
						} else if (iv.format() == ImageFormat::RGB || iv.format() == ImageFormat::RGBA) {
							lum = RGBToLum(src[0], src[1], src[2]);
						} else if (iv.format() == ImageFormat::BGR || iv.format() == ImageFormat::BGRA) {
							lum = RGBToLum(src[2], src[1], src[0]);
						} else {
							lum = RGBToLum(src[RedIndex(iv.format())], src[GreenIndex(iv.format())], src[BlueIndex(iv.format())]);
						}
						sum += lum;
						sumSq += lum * lum;
						minLum = std::min(minLum, (int)lum);
						maxLum = std::max(maxLum, (int)lum);
						count++;
					}
				}

				if (count > 0) {
					int mean = sum / count;
					int variance = (sumSq / count) - (mean * mean);
					int range = maxLum - minLum;

					// Count this block if it has high variance OR high range
					if (variance > varianceThreshold || range > 40) {
						highVariancePixels += count;
					}
				}
			}
		}

		rankedRegions.push_back({rect, highVariancePixels});
	}

	// Sort by high-variance area (highest first) - prioritizes QR patterns over text
	std::sort(rankedRegions.begin(), rankedRegions.end(),
		[](const RegionWithScore& a, const RegionWithScore& b) {
			return a.highVarianceArea > b.highVarianceArea;
		});

	// Return top 3 regions with highest variance area
	std::vector<SimpleRect> topRegions;
	for (size_t i = 0; i < std::min(size_t(3), rankedRegions.size()); ++i) {
		topRegions.push_back(rankedRegions[i].rect);
	}

	return topRegions;
}

// Calculate percentage of dark ("black") pixels in a region
// Returns value 0.0-1.0 representing fraction of pixels below threshold
static double CalculateBlackPixelRatio(const ImageView& iv, const SimpleRect& rect)
{
	const int blackThreshold = 128; // Pixels below this are considered "black"
	int blackCount = 0;
	int totalCount = 0;

	for (int y = rect.y; y < rect.y + rect.height && y < iv.height(); ++y) {
		for (int x = rect.x; x < rect.x + rect.width && x < iv.width(); ++x) {
			auto* src = iv.data(x, y);
			uint8_t lum;
			if (iv.format() == ImageFormat::Lum) {
				lum = *src;
			} else if (iv.format() == ImageFormat::RGB || iv.format() == ImageFormat::RGBA) {
				lum = RGBToLum(src[0], src[1], src[2]);
			} else if (iv.format() == ImageFormat::BGR || iv.format() == ImageFormat::BGRA) {
				lum = RGBToLum(src[2], src[1], src[0]);
			} else {
				lum = RGBToLum(src[RedIndex(iv.format())], src[GreenIndex(iv.format())], src[BlueIndex(iv.format())]);
			}

			if (lum < blackThreshold) {
				blackCount++;
			}
			totalCount++;
		}
	}

	if (totalCount == 0) return 0.0;
	return (double)blackCount / (double)totalCount;
}

// Optimize region by cropping to minimize black pixel count
// This removes black background around white QR codes
static SimpleRect OptimizeRegionBounds(const ImageView& iv, const SimpleRect& initialRect)
{
	SimpleRect rect = initialRect;
	const int minSize = 50; // Don't crop smaller than 50x50 (QR codes need space)
	const int cropStep = 3; // Crop 3 pixels at a time (less aggressive)

	if (rect.width <= minSize || rect.height <= minSize) return rect;

	// Calculate initial black pixel ratio
	double blackRatio = CalculateBlackPixelRatio(iv, rect);
	bool improved = true;
	int iterations = 0;
	const int maxIterations = 20; // Safety limit to prevent over-cropping

	// Keep cropping while black pixel ratio decreases (removing black background)
	while (improved && rect.width > minSize && rect.height > minSize && iterations < maxIterations) {
		improved = false;
		SimpleRect bestRect = rect;
		double bestBlackRatio = blackRatio;
		iterations++;

		// Try cropping from each side
		// Crop from left
		if (rect.width - cropStep >= minSize) {
			SimpleRect candidate = rect;
			candidate.x += cropStep;
			candidate.width -= cropStep;
			double candBlackRatio = CalculateBlackPixelRatio(iv, candidate);
			if (candBlackRatio < bestBlackRatio) {
				bestBlackRatio = candBlackRatio;
				bestRect = candidate;
				improved = true;
			}
		}

		// Crop from right
		if (rect.width - cropStep >= minSize) {
			SimpleRect candidate = rect;
			candidate.width -= cropStep;
			double candBlackRatio = CalculateBlackPixelRatio(iv, candidate);
			if (candBlackRatio < bestBlackRatio) {
				bestBlackRatio = candBlackRatio;
				bestRect = candidate;
				improved = true;
			}
		}

		// Crop from top
		if (rect.height - cropStep >= minSize) {
			SimpleRect candidate = rect;
			candidate.y += cropStep;
			candidate.height -= cropStep;
			double candBlackRatio = CalculateBlackPixelRatio(iv, candidate);
			if (candBlackRatio < bestBlackRatio) {
				bestBlackRatio = candBlackRatio;
				bestRect = candidate;
				improved = true;
			}
		}

		// Crop from bottom
		if (rect.height - cropStep >= minSize) {
			SimpleRect candidate = rect;
			candidate.height -= cropStep;
			double candBlackRatio = CalculateBlackPixelRatio(iv, candidate);
			if (candBlackRatio < bestBlackRatio) {
				bestBlackRatio = candBlackRatio;
				bestRect = candidate;
				improved = true;
			}
		}

		// If we found a better crop, apply it and continue
		if (improved) {
			rect = bestRect;
			blackRatio = bestBlackRatio;
		}
	}

	return rect;
}

// Replace black background pixels with white
// This helps decode white QR codes on black backgrounds
static void ReplaceBlackWithWhite(LumImage& img)
{
	const int blackThreshold = 128; // Pixels below this are considered background

	for (int i = 0; i < img.width() * img.height(); ++i) {
		uint8_t lum = img.data()[i];
		// If pixel is black (below threshold), replace with white
		if (lum < blackThreshold) {
			img.data()[i] = 255; // White
		}
	}
}

// Apply inversion filter to a region: simple inversion for black backgrounds
static void ApplyInversionFilter(LumImage& img)
{
	for (int i = 0; i < img.width() * img.height(); ++i) {
		// Simple inversion: white QR on black -> black QR on white
		img.data()[i] = 255 - img.data()[i];
	}
}

// Random number generator helpers
static uint32_t g_floodFillSeed = 12345;
static uint32_t FloodFillRandom()
{
	g_floodFillSeed = g_floodFillSeed * 1103515245 + 12345;
	return (g_floodFillSeed / 65536) % 32768;
}

// Get random point with optional edge bias
// If edgeBias is true, uses edge-favoring distribution for both x and y independently
// If edgeBias is false, uses uniform random distribution
static std::pair<int, int> GetRandomPoint(int width, int height, bool edgeBias = true)
{
	int x, y;

	if (edgeBias) {
		// Apply edge-favoring distribution to x and y independently
		// Weight: edges have 9x probability compared to center

		// Select X coordinate with edge bias
		int totalWeightX = 19; // 2 edges (left+right) * 9 weight + 1 center * 1 weight
		int choiceX = FloodFillRandom() % totalWeightX;

		if (choiceX < 9) {
			// Left edge (weight 9) - leftmost 25% of width
			x = FloodFillRandom() % (width / 4);
		} else if (choiceX < 18) {
			// Right edge (weight 9) - rightmost 25% of width
			x = width - 1 - (FloodFillRandom() % (width / 4));
		} else {
			// Center (weight 1) - middle 50% of width
			x = width / 4 + (FloodFillRandom() % (width / 2));
		}

		// Select Y coordinate with same edge bias
		int totalWeightY = 19; // 2 edges (top+bottom) * 9 weight + 1 center * 1 weight
		int choiceY = FloodFillRandom() % totalWeightY;

		if (choiceY < 9) {
			// Top edge (weight 9) - topmost 25% of height
			y = FloodFillRandom() % (height / 4);
		} else if (choiceY < 18) {
			// Bottom edge (weight 9) - bottommost 25% of height
			y = height - 1 - (FloodFillRandom() % (height / 4));
		} else {
			// Center (weight 1) - middle 50% of height
			y = height / 4 + (FloodFillRandom() % (height / 2));
		}
	} else {
		// Uniform random distribution
		x = FloodFillRandom() % width;
		y = FloodFillRandom() % height;
	}

	return {x, y};
}

// Check if two colors are similar (fuzzy matching)
static bool ColorsAreSimilar(uint8_t lum1, uint8_t lum2, int threshold = 17)
{
	return std::abs(static_cast<int>(lum1) - static_cast<int>(lum2)) <= threshold;
}

// Fuzzy flood fill algorithm with multiple starting points
// If point is dark, fill with white. If point is light, fill with black.
// C: number of starting points to use (all with same color threshold)
static void FuzzyFloodFill(LumImage& img, int startX, int startY, int threshold = 17, int pointCount = 1, bool edgeBias = true)
{
	if (startX < 0 || startX >= img.width() || startY < 0 || startY >= img.height())
		return;

	const int width = img.width();
	const int height = img.height();

	uint8_t startLum = img.data()[startY * width + startX];

	// Determine fill color based on starting luminance
	uint8_t fillColor;
	if (startLum < 85) {
		// Dark pixel -> fill with white
		fillColor = 255;
	} else if (startLum > 170) {
		// Light pixel -> fill with black
		fillColor = 0;
	} else {
		// Medium gray -> randomly choose black or white
		fillColor = (FloodFillRandom() % 2) ? 255 : 0;
	}

	// Use scanline flood fill algorithm for proper area filling
	std::vector<std::pair<int, int>> stack;
	std::vector<bool> filled(width * height, false);

	// Add first point to stack
	stack.push_back({startX, startY});

	// Select C-1 additional points with similar color to the first point
	// Use same probability distribution (edge-biased or uniform) as first point
	if (pointCount > 1) {
		int pointsAdded = 1;
		int maxAttempts = pointCount * 10; // Try up to 10x to find similar-colored points
		int attempts = 0;

		while (pointsAdded < pointCount && attempts < maxAttempts) {
			auto [randX, randY] = GetRandomPoint(width, height, edgeBias);
			uint8_t randLum = img.data()[randY * width + randX];

			// Check if this point has similar color to the starting point
			if (ColorsAreSimilar(randLum, startLum, threshold)) {
				stack.push_back({randX, randY});
				pointsAdded++;
			}
			attempts++;
		}
	}

	while (!stack.empty()) {
		auto [x, y] = stack.back();
		stack.pop_back();

		// Find the leftmost pixel in this scanline
		int left = x;
		while (left > 0) {
			int idx = y * width + (left - 1);
			if (filled[idx])
				break;
			uint8_t lum = img.data()[idx];
			if (!ColorsAreSimilar(lum, startLum, threshold))
				break;
			left--;
		}

		// Find the rightmost pixel in this scanline
		int right = x;
		while (right < width - 1) {
			int idx = y * width + (right + 1);
			if (filled[idx])
				break;
			uint8_t lum = img.data()[idx];
			if (!ColorsAreSimilar(lum, startLum, threshold))
				break;
			right++;
		}

		// Fill the scanline from left to right
		bool spanAbove = false;
		bool spanBelow = false;

		for (int i = left; i <= right; i++) {
			int idx = y * width + i;
			img.data()[idx] = fillColor;
			filled[idx] = true;

			// Check pixel above
			if (y > 0) {
				int idxAbove = (y - 1) * width + i;
				if (!filled[idxAbove]) {
					uint8_t lumAbove = img.data()[idxAbove];
					if (ColorsAreSimilar(lumAbove, startLum, threshold)) {
						if (!spanAbove) {
							stack.push_back({i, y - 1});
							spanAbove = true;
						}
					} else {
						spanAbove = false;
					}
				} else {
					spanAbove = false;
				}
			}

			// Check pixel below
			if (y < height - 1) {
				int idxBelow = (y + 1) * width + i;
				if (!filled[idxBelow]) {
					uint8_t lumBelow = img.data()[idxBelow];
					if (ColorsAreSimilar(lumBelow, startLum, threshold)) {
						if (!spanBelow) {
							stack.push_back({i, y + 1});
							spanBelow = true;
						}
					} else {
						spanBelow = false;
					}
				} else {
					spanBelow = false;
				}
			}
		}
	}
}

ImageView SetupLumImageView(ImageView iv, LumImage& lum, const ReaderOptions& opts)
{
	if (iv.format() == ImageFormat::None)
		throw std::invalid_argument("Invalid image format");

	if (opts.binarizer() == Binarizer::GlobalHistogram || opts.binarizer() == Binarizer::LocalAverage) {
		// manually spell out the 3 most common pixel formats to get at least gcc to vectorize the code
		if (iv.format() == ImageFormat::RGB && iv.pixStride() == 3) {
			lum = ExtractLum(iv, [](const uint8_t* src) { return RGBToLum(src[0], src[1], src[2]); });
		} else if (iv.format() == ImageFormat::RGBA && iv.pixStride() == 4) {
			lum = ExtractLum(iv, [](const uint8_t* src) { return RGBToLum(src[0], src[1], src[2]); });
		} else if (iv.format() == ImageFormat::BGR && iv.pixStride() == 3) {
			lum = ExtractLum(iv, [](const uint8_t* src) { return RGBToLum(src[2], src[1], src[0]); });
		} else if (iv.format() != ImageFormat::Lum) {
			lum = ExtractLum(iv, [r = RedIndex(iv.format()), g = GreenIndex(iv.format()), b = BlueIndex(iv.format())](
									 const uint8_t* src) { return RGBToLum(src[r], src[g], src[b]); });
		} else if (iv.pixStride() != 1) {
			// GlobalHistogram and LocalAverage need dense line memory layout
			lum = ExtractLum(iv, [](const uint8_t* src) { return *src; });
		}
		if (lum.data())
			return lum;
	}
	return iv;
}

std::unique_ptr<BinaryBitmap> CreateBitmap(ZXing::Binarizer binarizer, const ImageView& iv)
{
	switch (binarizer) {
	case Binarizer::BoolCast: return std::make_unique<ThresholdBinarizer>(iv, 0);
	case Binarizer::FixedThreshold: return std::make_unique<ThresholdBinarizer>(iv, 127);
	case Binarizer::GlobalHistogram: return std::make_unique<GlobalHistogramBinarizer>(iv);
	case Binarizer::LocalAverage: return std::make_unique<HybridBinarizer>(iv);
	}
	return {}; // silence gcc warning
}

// Try flood fill preprocessing with multiple random attempts
// Returns the modified bitmap and result if successful, otherwise returns empty result
static std::pair<Barcode, bool> TryFloodFillPreprocessingHelper(const ImageView& _iv, const ReaderOptions& opts,
                                      MultiFormatReader& reader, DebugInfo* debug, int maxAttempts = 9)
{
	if (debug) {
		debug->usedFloodFill = true;
		debug->floodFillWidth = 0;
		debug->floodFillHeight = 0;
	}

	int threshold = opts.floodFillThreshold();

	for (int attempt = 0; attempt < maxAttempts; ++attempt) {
		// Create a working copy of the image
		LumImage workingCopy;
		ImageView iv = SetupLumImageView(_iv, workingCopy, opts);

		// If workingCopy was created, use it; otherwise create from original
		if (!workingCopy.data()) {
			workingCopy = LumImage(iv.width(), iv.height());
			std::memcpy(workingCopy.data(), iv.data(), iv.width() * iv.height());
		} else {
			// Already have a copy, make another one
			LumImage temp(workingCopy.width(), workingCopy.height());
			std::memcpy(temp.data(), workingCopy.data(), workingCopy.width() * workingCopy.height());
			workingCopy = std::move(temp);
		}

		// Get random point (edge-biased or uniform based on option)
		bool edgeBias = opts.floodFillEdgeBias();
		auto [randX, randY] = GetRandomPoint(workingCopy.width(), workingCopy.height(), edgeBias);

		// Apply fuzzy flood fill from multiple points
		// This fills from C different points of the same color simultaneously
		int pointCount = opts.floodFillPointCount();
		pointCount = (pointCount > 0 && pointCount <= 10) ? pointCount : 3; // Clamp to 1-10 range
		FuzzyFloodFill(workingCopy, randX, randY, threshold, pointCount, edgeBias);

		// Save flood fill image to debug info if detection fails
		// We'll save it after trying detection
		std::vector<uint8_t> imageCopy;
		if (debug) {
			int size = workingCopy.width() * workingCopy.height();
			imageCopy.resize(size);
			std::memcpy(imageCopy.data(), workingCopy.data(), size);
			if (debug->floodFillWidth == 0) {
				debug->floodFillWidth = workingCopy.width();
				debug->floodFillHeight = workingCopy.height();
			}
		}

		// Try detection on the modified image
		auto bitmap = CreateBitmap(opts.binarizer(), workingCopy);
		bool wasInverted = false;

		// Try both normal and inverted
		for (int invert = 0; invert <= static_cast<int>(opts.tryInvert()); ++invert) {
			if (invert)
				bitmap->invert();

			auto r = reader.read(*bitmap);
			if (r.isValid()) {
				wasInverted = bitmap->inverted();
				// Return result and inverted status, let ReadBarcodes set private fields
				return {std::move(r), wasInverted};
			}
		}

		// Detection failed, save the image to debug info
		if (debug && !imageCopy.empty()) {
			debug->floodFillImages.push_back(std::move(imageCopy));
		}
	}

	return {{}, false};
}

Barcode ReadBarcode(const ImageView& _iv, const ReaderOptions& opts)
{
	return FirstOrDefault(ReadBarcodes(_iv, ReaderOptions(opts).setMaxNumberOfSymbols(1)));
}

Barcodes ReadBarcodes(const ImageView& _iv, const ReaderOptions& opts, void* debugOut)
{
	DebugInfo* debug = static_cast<DebugInfo*>(debugOut);
	if (sizeof(PatternType) < 4 && (_iv.width() > 0xffff || _iv.height() > 0xffff))
		throw std::invalid_argument("Maximum image width/height is 65535");

	if (!_iv.data() || _iv.width() * _iv.height() == 0)
		throw std::invalid_argument("ImageView is null/empty");

	LumImage lum;
	ImageView iv = SetupLumImageView(_iv, lum, opts);

	MultiFormatReader reader(opts);

	if (opts.isPure())
		return {reader.read(*CreateBitmap(opts.binarizer(), iv)).setReaderOptions(opts)};

	std::unique_ptr<MultiFormatReader> closedReader;
#ifdef ZXING_EXPERIMENTAL_API
	auto formatsBenefittingFromClosing = BarcodeFormat::Aztec | BarcodeFormat::DataMatrix | BarcodeFormat::QRCode | BarcodeFormat::MicroQRCode;
	ReaderOptions closedOptions = opts;
	if (opts.tryDenoise() && opts.hasFormat(formatsBenefittingFromClosing) && _iv.height() >= 3) {
		closedOptions.setFormats((opts.formats().empty() ? BarcodeFormat::Any : opts.formats()) & formatsBenefittingFromClosing);
		closedReader = std::make_unique<MultiFormatReader>(closedOptions);
	}
#endif
	LumImagePyramid pyramid(iv, opts.downscaleThreshold() * opts.tryDownscale(), opts.downscaleFactor());

	Barcodes res;
	int maxSymbols = opts.maxNumberOfSymbols() ? opts.maxNumberOfSymbols() : INT_MAX;
	for (auto&& iv : pyramid.layers) {
		auto bitmap = CreateBitmap(opts.binarizer(), iv);
		for (int close = 0; close <= (closedReader ? 1 : 0); ++close) {
			if (close) {
				// if we already inverted the image in the first round, we need to undo that first
				if (bitmap->inverted())
					bitmap->invert();
				bitmap->close();
			}

			// Always try both normal and inverted when tryInvert is enabled
			for (int invert = 0; invert <= static_cast<int>(opts.tryInvert()); ++invert) {
				if (invert)
					bitmap->invert();
				auto rs = (close ? *closedReader : reader).readMultiple(*bitmap, maxSymbols);
				for (auto& r : rs) {
					if (iv.width() != _iv.width())
						r.setPosition(Scale(r.position(), _iv.width() / iv.width()));
					if (!Contains(res, r)) {
						r.setReaderOptions(opts);
						r.setIsInverted(bitmap->inverted());
						res.push_back(std::move(r));
						--maxSymbols;
					}
				}
				if (maxSymbols <= 0)
					return res;
			}
		}
	}

	// FLOOD FILL PREPROCESSING: Try flood fill before region detection if enabled
	if (res.empty() && opts.tryFloodFill() && opts.hasFormat(BarcodeFormat::QRCode | BarcodeFormat::MicroQRCode)) {
		auto [result, wasInverted] = TryFloodFillPreprocessingHelper(_iv, opts, reader, debug, opts.maxFloodFillCount());
		if (result.isValid()) {
			result.setReaderOptions(opts);
			result.setIsInverted(wasInverted);
			res.push_back(std::move(result));
			return res; // Successfully found barcode with flood fill
		}
	}

	// FALLBACK: If no QR codes found and tryFindVarianceRegions enabled, try region-based detection
	if (res.empty() && opts.tryFindVarianceRegions() && opts.hasFormat(BarcodeFormat::QRCode | BarcodeFormat::MicroQRCode)) {
		auto qrRegions = DetectQRCodeRegions(_iv);

		// Populate debug info
		if (debug) {
			debug->usedFallback = true;
			debug->regionsProcessed = 0;

			// If no regions detected, add a default full-image rect for debugging
			if (qrRegions.empty()) {
				SimpleRect fullImage;
				fullImage.x = 0;
				fullImage.y = 0;
				fullImage.width = _iv.width();
				fullImage.height = _iv.height();
				debug->detectedRegions.push_back(fullImage);
			} else {
				debug->detectedRegions = qrRegions;
			}
		}

		// Process regions in order from highest score to lowest
		// Return immediately on first successful decode
		for (auto region : qrRegions) {
			if (debug) debug->regionsProcessed++;

			// Skip optimization - use raw detected region
			// Optimization might be removing QR code parts

			// Extract the region
			LumImage regionStorage;
			ExtractRegion(_iv, region, regionStorage);

			// Try multiple strategies with different binarizers
			// This is important because different binarizers work better for different contrast scenarios
			std::vector<Binarizer> binarizerStrategies = {
				Binarizer::LocalAverage,      // HybridBinarizer - best for varying illumination
				Binarizer::GlobalHistogram,   // Adaptive thresholding
				Binarizer::FixedThreshold     // Simple threshold
			};

			for (auto binarizer : binarizerStrategies) {
				// Try 1: Full inversion (white QR on black -> black QR on white)
				// This is most effective for white QR codes on black backgrounds
				LumImage invertedStorage(regionStorage.width(), regionStorage.height());
				std::memcpy(invertedStorage.data(), regionStorage.data(), regionStorage.width() * regionStorage.height());
				ApplyInversionFilter(invertedStorage);
				auto regionBitmap = CreateBitmap(binarizer, invertedStorage);
				auto r = reader.read(*regionBitmap);
				if (r.isValid()) {
					r.setPosition(Translate(r.position(), region.x, region.y));
					r.setReaderOptions(opts);
					r.setIsInverted(true);
					res.push_back(std::move(r));
					return res; // Return immediately on success
				}

				// Try 2: Normal decode (no modification)
				regionBitmap = CreateBitmap(binarizer, regionStorage);
				r = reader.read(*regionBitmap);
				if (r.isValid()) {
					r.setPosition(Translate(r.position(), region.x, region.y));
					r.setReaderOptions(opts);
					res.push_back(std::move(r));
					return res; // Return immediately on success
				}
			}
		}
	}

	// Always provide debug info when no detection happened (for diagnostics)
	if (res.empty() && debug && debug->detectedRegions.empty()) {
		SimpleRect fullImage;
		fullImage.x = 0;
		fullImage.y = 0;
		fullImage.width = _iv.width();
		fullImage.height = _iv.height();
		debug->detectedRegions.push_back(fullImage);
	}

	return res;
}

#else // ZXING_READERS

Barcode ReadBarcode(const ImageView&, const ReaderOptions&)
{
	throw std::runtime_error("This build of zxing-cpp does not support reading barcodes.");
}

Barcodes ReadBarcodes(const ImageView&, const ReaderOptions&)
{
	throw std::runtime_error("This build of zxing-cpp does not support reading barcodes.");
}

#endif // ZXING_READERS

} // ZXing
