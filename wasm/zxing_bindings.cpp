#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "MultiFormatReader.h"
#include "ReadBarcode.h"
#include "TextUtfEncoding.h"

using namespace emscripten;
using namespace ZXing;

struct ReadResult {
    std::string text;
    std::string format;
    bool isValid;
    std::string error;
};

ReadResult readBarcode(const std::string& imageData, int width, int height, 
                       bool tryHarder, bool tryRotate, bool tryInvert) {
    ReadResult result;
    try {
        DecodeHints hints;
        hints.setTryHarder(tryHarder);
        hints.setTryRotate(tryRotate);
        hints.setTryInvert(tryInvert);
        hints.setFormats(BarcodeFormat::Any);
        
        ImageView image{reinterpret_cast<const uint8_t*>(imageData.data()), 
                       width, height, ImageFormat::Lum};
        auto barcodeResult = ReadBarcode(image, hints);
        
        if (barcodeResult.isValid()) {
            result.text = TextUtfEncoding::ToUtf8(barcodeResult.text());
            result.format = ToString(barcodeResult.format());
            result.isValid = true;
        } else {
            result.isValid = false;
            result.error = "No barcode found";
        }
    } catch (const std::exception& e) {
        result.isValid = false;
        result.error = e.what();
    }
    return result;
}

EMSCRIPTEN_BINDINGS(zxing_wasm) {
    value_object<ReadResult>("ReadResult")
        .field("text", &ReadResult::text)
        .field("format", &ReadResult::format)
        .field("isValid", &ReadResult::isValid)
        .field("error", &ReadResult::error);
    
    function("readBarcode", &readBarcode);
}
