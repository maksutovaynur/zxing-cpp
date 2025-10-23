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
// Uses local variance analysis to find structured patterns
static std::vector<SimpleRect> DetectQRCodeRegions(const ImageView& iv)
{
	std::vector<SimpleRect> regions;
	const int width = iv.width();
	const int height = iv.height();
	const int blockSize = 20; // Smaller blocks to catch smaller QR codes
	const int varianceThreshold = 500; // Lower threshold to be more sensitive

	// Grid to mark high-contrast blocks
	std::vector<bool> highContrastBlocks((width / blockSize + 1) * (height / blockSize + 1), false);

	// Scan image in blocks to find high-variance areas (QR codes have high contrast)
	for (int by = 0; by < height - blockSize; by += blockSize) {
		for (int bx = 0; bx < width - blockSize; bx += blockSize) {
			// Calculate variance in this block
			int sum = 0;
			int sumSq = 0;
			int count = 0;

			// Sample every pixel for better accuracy
			for (int y = by; y < by + blockSize && y < height; ++y) {
				for (int x = bx; x < bx + blockSize && x < width; ++x) {
					uint8_t lum;
					auto* src = iv.data(x, y);
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
					count++;
				}
			}

			if (count > 0) {
				int mean = sum / count;
				int variance = (sumSq / count) - (mean * mean);

				// Mark high-variance blocks (likely QR codes)
				if (variance > varianceThreshold) {
					int blockIdx = (by / blockSize) * (width / blockSize + 1) + (bx / blockSize);
					highContrastBlocks[blockIdx] = true;
				}
			}
		}
	}

	// Find connected components of high-contrast blocks and create bounding boxes
	std::vector<bool> visited = highContrastBlocks;
	for (int by = 0; by < height / blockSize; by++) {
		for (int bx = 0; bx < width / blockSize; bx++) {
			int blockIdx = by * (width / blockSize + 1) + bx;
			if (highContrastBlocks[blockIdx] && !visited[blockIdx]) {
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
						if (nx >= 0 && nx < width / blockSize && ny >= 0 && ny < height / blockSize) {
							int nidx = ny * (width / blockSize + 1) + nx;
							if (highContrastBlocks[nidx] && !visited[nidx]) {
								visited[nidx] = true;
								stack.push_back({nx, ny});
							}
						}
					}
				}

				// Create bounding rectangle with padding
				const int padding = blockSize * 2; // Extra padding around QR code
				SimpleRect rect;
				rect.x = std::max(0, minX * blockSize - padding);
				rect.y = std::max(0, minY * blockSize - padding);
				rect.width = std::min(width - rect.x, (maxX - minX + 1) * blockSize + 2 * padding);
				rect.height = std::min(height - rect.y, (maxY - minY + 1) * blockSize + 2 * padding);

				// Only add reasonably sized regions (likely to be QR codes)
				if (rect.width >= blockSize * 2 && rect.height >= blockSize * 2 &&
				    rect.width <= width * 0.8 && rect.height <= height * 0.8) {
					regions.push_back(rect);
				}
			}
		}
	}

	return regions;
}

// Apply inversion filter to a region: (1-r)*(1-g)*(1-b)
static void ApplyInversionFilter(LumImage& img)
{
	for (int i = 0; i < img.width() * img.height(); ++i) {
		uint8_t lum = img.data()[i];
		// Apply (1-L)^3 filter (grayscale equivalent of (1-r)*(1-g)*(1-b))
		float normalized = lum / 255.0f;
		float inverted = 1.0f - normalized;
		float filtered = inverted * inverted * inverted;
		img.data()[i] = static_cast<uint8_t>(filtered * 255.0f);
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

Barcode ReadBarcode(const ImageView& _iv, const ReaderOptions& opts)
{
	return FirstOrDefault(ReadBarcodes(_iv, ReaderOptions(opts).setMaxNumberOfSymbols(1)));
}

Barcodes ReadBarcodes(const ImageView& _iv, const ReaderOptions& opts)
{
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

	// FALLBACK: If no QR codes found and tryInvert enabled, try region-based detection
	if (res.empty() && opts.tryInvert() && opts.hasFormat(BarcodeFormat::QRCode | BarcodeFormat::MicroQRCode)) {
		auto qrRegions = DetectQRCodeRegions(_iv);

		for (const auto& region : qrRegions) {
			// Extract the region
			LumImage regionStorage;
			auto regionView = ExtractRegion(_iv, region, regionStorage);
			auto regionBitmap = CreateBitmap(opts.binarizer(), regionView);

			// Try normal decode
			auto r = reader.read(*regionBitmap);
			if (r.isValid()) {
				// Adjust position to original image coordinates
				r.setPosition(Translate(r.position(), region.x, region.y));
				r.setReaderOptions(opts);
				res.push_back(std::move(r));
				if (--maxSymbols <= 0)
					return res;
				continue;
			}

			// Try with inversion filter applied
			ApplyInversionFilter(regionStorage);
			regionBitmap = CreateBitmap(opts.binarizer(), regionStorage);
			r = reader.read(*regionBitmap);
			if (r.isValid()) {
				r.setPosition(Translate(r.position(), region.x, region.y));
				r.setReaderOptions(opts);
				r.setIsInverted(true);
				res.push_back(std::move(r));
				if (--maxSymbols <= 0)
					return res;
			}
		}
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
