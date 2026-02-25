#include "event_based_camera/filter_pipeline.hpp"

#include <stdexcept>

void FilterPipeline::init(uint16_t width, uint16_t height, const Config& config) {
    if (config.enable_denoising) {
        temporal_filter_.reset(edvs_temporal_filter_create(
            width, height, config.denoise_threshold_us));
        if (!temporal_filter_) {
            throw std::runtime_error("Failed to create temporal denoising filter");
        }
    }

    if (config.enable_hot_pixel) {
        hot_pixel_filter_.reset(edvs_hot_pixel_filter_create(
            width, height, config.hot_pixel_window_us, config.hot_pixel_max_rate));
        if (!hot_pixel_filter_) {
            temporal_filter_.reset();  // clean up on partial failure
            throw std::runtime_error("Failed to create hot pixel filter");
        }
    }
}

bool FilterPipeline::process(const edvs::Event* event) {
    if (!event) {
        return false;
    }
    if (temporal_filter_) {
        if (!edvs_temporal_filter_process(temporal_filter_.get(), event)) {
            return false;
        }
    }
    if (hot_pixel_filter_) {
        if (!edvs_hot_pixel_filter_process(hot_pixel_filter_.get(), event)) {
            return false;
        }
    }
    return true;
}

void FilterPipeline::reset() {
    temporal_filter_.reset();
    hot_pixel_filter_.reset();
}
