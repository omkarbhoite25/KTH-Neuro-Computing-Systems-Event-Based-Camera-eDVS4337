#include "event_based_camera/libcaer_edvs.hpp"

#include <stdexcept>

CameraInfo LibcaerEdvs::open(const std::string& port, uint32_t baud_rate) {
    device_ = std::make_unique<libcaer::devices::edvs>(1, port.c_str(), baud_rate);

    struct caer_edvs_info info = device_->infoGet();

    CameraInfo cam_info;
    cam_info.width = static_cast<uint16_t>(info.dvsSizeX);
    cam_info.height = static_cast<uint16_t>(info.dvsSizeY);
    cam_info.device_id = info.deviceID;
    cam_info.is_master = info.deviceIsMaster;
    return cam_info;
}

void LibcaerEdvs::startStream() {
    if (!device_) {
        throw std::runtime_error("Cannot start stream: device not opened");
    }
    device_->sendDefaultConfig();
    device_->dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);
    device_->configSet(CAER_HOST_CONFIG_DATAEXCHANGE,
                       CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
}

void LibcaerEdvs::stopStream() noexcept {
    try {
        if (device_) {
            device_->dataStop();
        }
    } catch (...) {
        // Swallow exceptions — stopStream is noexcept and called during cleanup.
    }
}

std::vector<edvs::Event> LibcaerEdvs::readEvents() {
    std::vector<edvs::Event> events;

    if (!device_) {
        return events;
    }

    std::unique_ptr<libcaer::events::EventPacketContainer> packet_container
        = device_->dataGet();

    if (!packet_container) {
        return events;
    }

    for (auto& packet : *packet_container) {
        if (!packet || packet->getEventType() != POLARITY_EVENT) {
            continue;
        }

        std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
            = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

        int32_t event_count = packet->getEventNumber();
        if (event_count <= 0) {
            continue;
        }

        events.reserve(events.size() + static_cast<size_t>(event_count));

        for (int32_t i = 0; i < event_count; ++i) {
            const libcaer::events::PolarityEvent& ev = (*polarity)[i];

            edvs::Event raw;
            raw.x = ev.getX();
            raw.y = ev.getY();
            raw.timestamp = ev.getTimestamp64(*polarity);
            raw.polarity = static_cast<int8_t>(2 * static_cast<int>(ev.getPolarity()) - 1);
            events.push_back(raw);
        }
    }

    return events;
}
