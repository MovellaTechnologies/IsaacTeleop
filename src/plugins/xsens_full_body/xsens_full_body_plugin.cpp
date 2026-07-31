// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "xsens_full_body_plugin.hpp"

#include <oxr/oxr_session.hpp>
#include <oxr_utils/os_time.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace plugins
{
namespace xsens_full_body
{

namespace
{

constexpr uint64_t kLogEveryFrames = 250; // ~1 s at the MVN 60/240 Hz stream / soak cadence

// Latency tap sampling interval (#3866): emit one ISAACLAT recv line per N delivered frames.
// 0 (the default) disables the tap, so a normal run's output is byte-identical to T4's.
uint64_t latencyTapInterval()
{
    const char* env = std::getenv("ISAACLAT_N");
    if (!env)
        return 0;
    const long n = std::atol(env);
    return (n > 0) ? static_cast<uint64_t>(n) : 0;
}

// FNV-1a 64-bit, for cheap per-frame payload fingerprinting (liveness evidence: distinct hashes
// over time prove the pose tracks the recording rather than a frozen/cached frame).
uint64_t fnv1a64(const uint8_t* data, size_t len)
{
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < len; ++i)
    {
        h ^= data[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

XsensFullBodyPlugin::XsensFullBodyPlugin(const std::string& collection_id, uint16_t port)
    : port_(port), collectionId_(collection_id), latencyTapN_(latencyTapInterval())
{
    establishSession();
}

void XsensFullBodyPlugin::establishSession()
{
    // Torn down first: once the runtime's IPC pipe breaks, only a full re-create works.
    pusher_.reset();
    session_.reset();

    session_ =
        std::make_shared<core::OpenXRSession>("XsensFullBodyPusher", core::SchemaPusher::get_required_extensions());
    // tensor_identifier MUST match the reader (core::XsensFullBodyTracker) or nothing is delivered.
    const core::SchemaPusherConfig config{ .collection_id = collectionId_,
                                           .max_flatbuffer_size = MAX_FLATBUFFER_SIZE,
                                           .tensor_identifier = "full_body_pose",
                                           .localized_name = "Xsens MVN Full Body",
                                           .app_name = "XsensFullBodyPusher" };
    pusher_.emplace(session_->get_handles(), config);
}

bool XsensFullBodyPlugin::recoverSession()
{
    static constexpr int backoffMs[MAX_SESSION_RECOVERY_ATTEMPTS] = { 500, 1000, 2000, 4000, 4000, 4000, 4000, 4000 }; // ~23.5 s
    static constexpr int sliceMs = 100;

    for (int attempt = 0; attempt < MAX_SESSION_RECOVERY_ATTEMPTS; ++attempt)
    {
        // Sliced so SIGINT/SIGTERM during a long outage still stops us promptly.
        for (int remaining = backoffMs[attempt]; remaining > 0; remaining -= sliceMs)
        {
            if (stop_ != nullptr && stop_->load(std::memory_order_relaxed))
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(remaining < sliceMs ? remaining : sliceMs));
        }

        try
        {
            establishSession();
            ++sessionRecoveries_;
            std::cout << "[XsensFullBodyPusher] session re-established after " << (attempt + 1)
                      << " attempt(s); resuming push" << std::endl;
            return true;
        }
        catch (const std::exception& e)
        {
            // Expected while the runtime is down; log only first and last so an outage cannot flood.
            if (attempt == 0 || attempt == MAX_SESSION_RECOVERY_ATTEMPTS - 1)
            {
                std::cerr << "[XsensFullBodyPusher] session re-establish attempt " << (attempt + 1) << "/"
                          << MAX_SESSION_RECOVERY_ATTEMPTS << " failed: " << e.what() << std::endl;
            }
        }
    }
    return false;
}

void XsensFullBodyPlugin::printStats() const
{
    const teleop::TeleopReceiverStats& s = receiver_.stats();
    std::cout << "[XsensFullBodyPusher] stopped. delivered=" << s.delivered.load() << " resets=" << s.resets.load()
              << " gaps=" << s.gapsTotal.load() << " droppedMalformed=" << s.droppedMalformed.load()
              << " droppedUnverified=" << s.droppedUnverified.load() << " droppedStale=" << s.droppedStale.load()
              << " droppedTruncated=" << s.droppedTruncated.load() << " warnedNonWholeMs=" << s.warnedNonWholeMs.load()
              << " warnedTimeBackward=" << s.warnedTimeBackward.load()
              << " socketRecoveries=" << s.socketRecoveries.load()
              << " socketRecoveryFailures=" << s.socketRecoveryFailures.load() << " pushFailures=" << pushFailures_
              << " sessionRecoveries=" << sessionRecoveries_ << std::endl;
}

void XsensFullBodyPlugin::onFrame(const teleop::TeleopFrame& frame)
{
    // Stamp the local common clock with the pusher host's CLOCK_MONOTONIC as late as possible
    // (arrival time at push) and forward the header's raw device time verbatim. The header's
    // sampleTimeNs is a different host's session-relative ms clock and must NOT be forwarded onto
    // the common clock. Sampled right before push_buffer so it does not absorb our own queueing.
    const int64_t localCommonNs = core::os_monotonic_now_ns();

    // push_buffer copies the bytes, so the borrowed payload lifetime is fine. It THROWS on failure:
    // a dead CloudXR runtime arrives as XR_ERROR_RUNTIME_FAILURE (-2), not XR_ERROR_SESSION_LOST.
    try
    {
        pusher_->push_buffer(frame.payload, frame.payloadLen, localCommonNs, frame.rawDeviceTimeNs);
    }
    catch (const std::exception& e)
    {
        ++pushFailures_;
        std::cerr << "[XsensFullBodyPusher] push failed at seq=" << frame.seq << ": " << e.what()
                  << " -- attempting session re-establish" << std::endl;
        if (!recoverSession())
        {
            // Rethrow so run() still prints the counters; this is the operator-restart case.
            throw std::runtime_error("XsensFullBodyPlugin: OpenXR session unrecoverable after " +
                                     std::to_string(MAX_SESSION_RECOVERY_ATTEMPTS) +
                                     " re-establish attempts -- restart the CloudXR runtime, then restart this pusher");
        }
        return; // recovered; drop this stale frame
    }

    // #3866 latency tap. recv_ns comes off the socket (receiver), push_ns is the stamp above, so
    // the gap is framing + verify + push. push_ns is also the join key to the reader-side line: it
    // is what the reader reads back as sample_time_local_common_clock.
    if (latencyTapN_ != 0 && (delivered_ % latencyTapN_) == 0)
    {
        std::cout << "ISAACLAT recv seq=" << frame.seq << " recv_ns=" << frame.recvMonotonicNs
                  << " push_ns=" << localCommonNs << " session=" << (frame.sessionStart ? 1 : 0) << std::endl;
    }

    // First frame of every session (startup or seq reset): header time vs push time.
    if (frame.sessionStart)
    {
        std::cout << "[XsensFullBodyPusher] session seq=" << frame.seq << " header.sampleTimeNs=" << frame.sampleTimeNs
                  << " header.rawDeviceTimeNs=" << frame.rawDeviceTimeNs << " pushed_at_monotonic_ns=" << localCommonNs
                  << std::endl;
    }

    // Periodic liveness/verbatim evidence: constant size re-proves the 784 B max_flatbuffer_size
    // sizing; distinct fnv1a64 across frames proves distinct live bytes (pose is tracking, not frozen).
    if (delivered_ % kLogEveryFrames == 0)
    {
        std::cout << "[XsensFullBodyPusher] delivered=" << delivered_ << " seq=" << frame.seq
                  << " size=" << frame.payloadLen << " fnv1a64=0x" << std::hex
                  << fnv1a64(frame.payload, frame.payloadLen) << std::dec << std::endl;
    }
    ++delivered_;
}

void XsensFullBodyPlugin::run(const std::atomic<bool>& stop)
{
    if (!receiver_.open(port_))
    {
        throw std::runtime_error("XsensFullBodyPlugin: failed to bind UDP port " + std::to_string(port_));
    }
    std::cout << "[XsensFullBodyPusher] listening on 0.0.0.0:" << port_
              << " -> push_buffer (collection tensor full_body_pose)" << std::endl;
    if (latencyTapN_ != 0)
    {
        std::cout << "ISAACLAT tap n=" << latencyTapN_ << " available=1" << std::endl;
    }

    // Borrowed for this call so onFrame's backoff can observe the stop flag.
    stop_ = &stop;

    // T5 recovery-matrix hook: N forced hard recvfrom errors. Unset in production.
    if (const char* inject = std::getenv("XSENS_TELEOP_INJECT_RECV_ERRORS"))
        receiver_.injectRecvErrors(ENOTCONN, std::atoi(inject));

    // Blocking: the receiver's sink calls onFrame() -> push_buffer() synchronously per verified
    // frame on this thread. Returns when stop is set or on an unrecoverable socket error.
    try
    {
        receiver_.run([this](const teleop::TeleopFrame& frame) { onFrame(frame); }, stop);
    }
    catch (...)
    {
        stop_ = nullptr;
        printStats();
        throw;
    }

    stop_ = nullptr;
    printStats();
}

} // namespace xsens_full_body
} // namespace plugins
