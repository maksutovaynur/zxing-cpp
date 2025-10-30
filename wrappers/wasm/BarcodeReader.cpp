/*
 * Copyright 2016 Nu-book Inc.
 * Copyright 2023 Axel Waggershauser
 */
// SPDX-License-Identifier: Apache-2.0

#include "ReadBarcode.h"

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <memory>
#include <stdexcept>
#include <string>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

using namespace ZXing;

struct DebugRect
{
	int x, y, width, height;
};

struct DebugInfo
{
	emscripten::val detectedRegions = emscripten::val::array(); // JavaScript array
	int regionsProcessed = 0;
	bool usedFallback = false;
	bool usedFloodFill = false;
	emscripten::val floodFillImages = emscripten::val::array(); // JavaScript array of Uint8Arrays
	int floodFillWidth = 0;
	int floodFillHeight = 0;
};

struct ReadResult
{
	std::string format{};
	std::string text{};
	emscripten::val bytes;
	std::string error{};
	Position position{};
	std::string symbologyIdentifier{};
	DebugInfo debug{};
};

std::vector<ReadResult> readBarcodes(ImageView iv, bool tryHarder, bool tryRotate, bool tryInvert, const std::string& format, int maxSymbols, bool tryFloodFill = false, int maxFloodFillCount = 5, bool tryFindVarianceRegions = false, int floodFillThreshold = 17, int floodFillPointCount = 3, bool floodFillEdgeBias = false)
{
	try {
		ReaderOptions opts;
		opts.setTryHarder(tryHarder);
		opts.setTryRotate(tryRotate);
		opts.setTryInvert(tryInvert);
		opts.setTryDownscale(tryHarder);
#ifdef ZXING_EXPERIMENTAL_API
		// Enable morphological operations (closing filter) for better boundary detection
		// This helps with white QR codes on black backgrounds
		opts.setTryDenoise(tryHarder);
#endif
		opts.setTryFloodFill(tryFloodFill);
		opts.setMaxFloodFillCount(maxFloodFillCount > 0 && maxFloodFillCount <= 15 ? maxFloodFillCount : 5);
		opts.setFloodFillThreshold(floodFillThreshold > 0 && floodFillThreshold <= 255 ? floodFillThreshold : 17);
		opts.setFloodFillPointCount(floodFillPointCount > 0 && floodFillPointCount <= 10 ? floodFillPointCount : 3);
		opts.setFloodFillEdgeBias(floodFillEdgeBias);
		opts.setTryFindVarianceRegions(tryFindVarianceRegions);
		opts.setFormats(BarcodeFormatsFromString(format));
		opts.setMaxNumberOfSymbols(maxSymbols);
//		opts.setReturnErrors(maxSymbols > 1);

		// Create debug info structure to capture region detection data
		// Note: This requires ReadBarcodes to accept DebugInfo* parameter
		// The implementation is in ReadBarcode.cpp and uses internal DebugInfo struct
		// We need to declare a compatible structure here
		struct InternalDebugInfo {
			struct SimpleRect { int x, y, width, height; };
			std::vector<SimpleRect> detectedRegions;
			int regionsProcessed = 0;
			bool usedFallback = false;
			bool usedFloodFill = false;
			std::vector<std::vector<uint8_t>> floodFillImages;
			int floodFillWidth = 0;
			int floodFillHeight = 0;
		};
		InternalDebugInfo internalDebug;

		auto barcodes = ReadBarcodes(iv, opts, reinterpret_cast<void*>(&internalDebug));

		std::vector<ReadResult> readResults{};
		readResults.reserve(std::max(barcodes.size(), size_t(1)));

		thread_local const emscripten::val Uint8Array = emscripten::val::global("Uint8Array");

		// Convert internal debug info to our WASM-compatible format
		DebugInfo debugInfo;
		debugInfo.regionsProcessed = internalDebug.regionsProcessed;
		debugInfo.usedFallback = internalDebug.usedFallback;
		debugInfo.usedFloodFill = internalDebug.usedFloodFill;
		debugInfo.floodFillWidth = internalDebug.floodFillWidth;
		debugInfo.floodFillHeight = internalDebug.floodFillHeight;

		// Create JavaScript array for regions
		debugInfo.detectedRegions = emscripten::val::array();
		for (size_t i = 0; i < internalDebug.detectedRegions.size(); ++i) {
			const auto& rect = internalDebug.detectedRegions[i];
			emscripten::val jsRect = emscripten::val::object();
			jsRect.set("x", rect.x);
			jsRect.set("y", rect.y);
			jsRect.set("width", rect.width);
			jsRect.set("height", rect.height);
			debugInfo.detectedRegions.call<void>("push", jsRect);
		}

		// Create JavaScript array for flood fill images
		debugInfo.floodFillImages = emscripten::val::array();
		for (size_t i = 0; i < internalDebug.floodFillImages.size(); ++i) {
			const auto& imageData = internalDebug.floodFillImages[i];
			emscripten::val jsImage = Uint8Array.new_(emscripten::typed_memory_view(imageData.size(), imageData.data()));
			debugInfo.floodFillImages.call<void>("push", jsImage);
		}

		for (auto&& barcode : barcodes) {
			const ByteArray& bytes = barcode.bytes();
			readResults.push_back({
				ToString(barcode.format()),
				barcode.text(),
				Uint8Array.new_(emscripten::typed_memory_view(bytes.size(), bytes.data())),
				ToString(barcode.error()),
				barcode.position(),
				barcode.symbologyIdentifier(),
				debugInfo
			});
		}

		// If no barcodes found but we have debug info, return an empty result with debug info
		int regionsLength = debugInfo.detectedRegions["length"].as<int>();
		if (readResults.empty() && (debugInfo.usedFallback || regionsLength > 0)) {
			readResults.push_back({"", "", {}, "No barcode found", {}, "", debugInfo});
		}

		return readResults;
	} catch (const std::exception& e) {
		return {{"", "", {}, e.what()}};
	} catch (...) {
		return {{"", "", {}, "Unknown error"}};
	}
	return {};
}

std::vector<ReadResult> readBarcodesFromImage(int bufferPtr, int bufferLength, bool tryHarder, bool tryRotate, bool tryInvert, std::string format, int maxSymbols, bool tryFloodFill = false, int maxFloodFillCount = 5, bool tryFindVarianceRegions = false, int floodFillThreshold = 17, int floodFillPointCount = 3, bool floodFillEdgeBias = false)
{
	int width, height, channels;
	std::unique_ptr<stbi_uc, void (*)(void*)> buffer(
		stbi_load_from_memory(reinterpret_cast<const unsigned char*>(bufferPtr), bufferLength, &width, &height, &channels, 1),
		stbi_image_free);
	if (buffer == nullptr)
		return {{"", "", {}, "Error loading image"}};

	return readBarcodes({buffer.get(), width, height, ImageFormat::Lum}, tryHarder, tryRotate, tryInvert, format, maxSymbols, tryFloodFill, maxFloodFillCount, tryFindVarianceRegions, floodFillThreshold, floodFillPointCount, floodFillEdgeBias);
}

ReadResult readBarcodeFromImage(int bufferPtr, int bufferLength, bool tryHarder, bool tryRotate, bool tryInvert, std::string format, bool tryFloodFill = false, int maxFloodFillCount = 5, bool tryFindVarianceRegions = false, int floodFillThreshold = 17, int floodFillPointCount = 3, bool floodFillEdgeBias = false)
{
	return FirstOrDefault(readBarcodesFromImage(bufferPtr, bufferLength, tryHarder, tryRotate, tryInvert, format, 1, tryFloodFill, maxFloodFillCount, tryFindVarianceRegions, floodFillThreshold, floodFillPointCount, floodFillEdgeBias));
}

std::vector<ReadResult> readBarcodesFromPixmap(int bufferPtr, int imgWidth, int imgHeight, bool tryHarder, bool tryRotate, bool tryInvert, std::string format, int maxSymbols, bool tryFloodFill = false, int maxFloodFillCount = 3, bool tryFindVarianceRegions = false, int floodFillThreshold = 17, int floodFillPointCount = 3, bool floodFillEdgeBias = false)
{
	return readBarcodes({reinterpret_cast<uint8_t*>(bufferPtr), imgWidth, imgHeight, ImageFormat::RGBA}, tryHarder, tryRotate, tryInvert, format, maxSymbols, tryFloodFill, maxFloodFillCount, tryFindVarianceRegions, floodFillThreshold, floodFillPointCount, floodFillEdgeBias);
}

ReadResult readBarcodeFromPixmap(int bufferPtr, int imgWidth, int imgHeight, bool tryHarder, bool tryRotate, bool tryInvert, std::string format, bool tryFloodFill = false, int maxFloodFillCount = 3, bool tryFindVarianceRegions = false, int floodFillThreshold = 17, int floodFillPointCount = 3, bool floodFillEdgeBias = false)
{
	return FirstOrDefault(readBarcodesFromPixmap(bufferPtr, imgWidth, imgHeight, tryHarder, tryRotate, tryInvert, format, 1, tryFloodFill, maxFloodFillCount, tryFindVarianceRegions, floodFillThreshold, floodFillPointCount, floodFillEdgeBias));
}

EMSCRIPTEN_BINDINGS(BarcodeReader)
{
	using namespace emscripten;

	// DebugInfo with JS array for detectedRegions and floodFillImages
	value_object<DebugInfo>("DebugInfo")
		.field("detectedRegions", &DebugInfo::detectedRegions)
		.field("regionsProcessed", &DebugInfo::regionsProcessed)
		.field("usedFallback", &DebugInfo::usedFallback)
		.field("usedFloodFill", &DebugInfo::usedFloodFill)
		.field("floodFillImages", &DebugInfo::floodFillImages)
		.field("floodFillWidth", &DebugInfo::floodFillWidth)
		.field("floodFillHeight", &DebugInfo::floodFillHeight);

	value_object<ReadResult>("ReadResult")
		.field("format", &ReadResult::format)
		.field("text", &ReadResult::text)
		.field("bytes", &ReadResult::bytes)
		.field("error", &ReadResult::error)
		.field("position", &ReadResult::position)
		.field("symbologyIdentifier", &ReadResult::symbologyIdentifier)
		.field("debug", &ReadResult::debug);

	value_object<ZXing::PointI>("Point").field("x", &ZXing::PointI::x).field("y", &ZXing::PointI::y);

	value_object<ZXing::Position>("Position")
		.field("topLeft", emscripten::index<0>())
		.field("topRight", emscripten::index<1>())
		.field("bottomRight", emscripten::index<2>())
		.field("bottomLeft", emscripten::index<3>());

	register_vector<ReadResult>("vector<ReadResult>");

	function("readBarcodeFromImage", &readBarcodeFromImage);
	function("readBarcodeFromPixmap", &readBarcodeFromPixmap);

	function("readBarcodesFromImage", &readBarcodesFromImage);
	function("readBarcodesFromPixmap", &readBarcodesFromPixmap);
};
