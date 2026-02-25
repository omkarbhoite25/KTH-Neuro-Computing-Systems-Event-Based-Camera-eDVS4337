#ifndef LIBCAER_EDVS_HPP
#define LIBCAER_EDVS_HPP

#include <memory>
#include <libcaercpp/devices/edvs.hpp>

#include "event_based_camera/evt_camera.hpp"

/// libcaer-based eDVS implementation of the EvtCamera interface.
class LibcaerEdvs : public EvtCamera {
public:
    LibcaerEdvs() = default;
    ~LibcaerEdvs() override = default;

    // Non-copyable
    LibcaerEdvs(const LibcaerEdvs&) = delete;
    LibcaerEdvs& operator=(const LibcaerEdvs&) = delete;

    CameraInfo open(const std::string& port, uint32_t baud_rate) override;
    void startStream() override;
    void stopStream() noexcept override;
    std::vector<edvs::Event> readEvents() override;

private:
    std::unique_ptr<libcaer::devices::edvs> device_;
};

#endif  // LIBCAER_EDVS_HPP
