// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Embedded from mvn_isaac_devtools/tools/teleop_receiver/ (#3863, T4 wire-through). Reformatted to
// the fork's clang-format-14 style on embed (SortIncludes + reflow) + this SPDX header added; the
// receiver LOGIC is unchanged (token stream identical). The dev/proving home (teleop_udp oracle +
// sanitizer self-tests, mvn tab style) stays in mvn_isaac_devtools.

#ifndef TELEOP_RECEIVER_H
#define TELEOP_RECEIVER_H

// Thin, path-independent UDP teleop receiver: MVN Studio's Isaac Teleop datagrams -> framed ->
// VERIFIED core.FullBodyPosePico payload bytes -> a sink. Plain C++ (no Qt, no XME) so it can later
// drop into the Isaac fork unchanged. The receiver is the trust boundary: every delivered payload
// has already passed picofullbody::verifyFullBodyPosePicoPayload, and this translation unit never
// roots a FlatBuffer itself (build-enforced by the grep gate in build.sh).

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace teleop
{
//! One delivered teleop frame handed to the sink. \c payload references VERIFIED
//! FullBodyPosePico bytes and is valid ONLY for the duration of the sink call: on the push path
//! it points into the receiver's recv buffer (no copy), so a consumer that needs to keep the
//! bytes must copy them (the pull adapter does exactly that).
struct TeleopFrame
{
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    uint64_t seq = 0;
    int64_t sampleTimeNs = 0; //!< header value, forwarded untouched (timestamp remap is downstream)
    int64_t rawDeviceTimeNs = 0; //!< header value, forwarded untouched
    bool sessionStart = false; //!< first frame of a session (startup or seq reset)
};

//! Sink invoked once per delivered (framed + verified) frame, synchronously on the receive
//! thread. Must be fast: it runs inside the receive loop and there is no backpressure.
using TeleopFrameSink = std::function<void(const TeleopFrame&)>;

//! Injectable log sink (the receiver rate-limits its calls). Defaults to stderr when empty.
using TeleopLogFn = std::function<void(const char*)>;

//! Per-category counters. All std::atomic<uint64_t> (relaxed): the pull route reads them from a
//! second thread while the receive thread writes them, so plain integers would be a data race.
struct TeleopReceiverStats
{
    std::atomic<uint64_t> delivered{ 0 }; //!< frames handed to the sink
    std::atomic<uint64_t> resets{ 0 }; //!< session boundaries (seq reset to 0)
    std::atomic<uint64_t> gapsTotal{ 0 }; //!< sum of missing seqs across all gaps
    std::atomic<uint64_t> droppedMalformed{ 0 }; //!< framing rejected (too short / magic / version / overrun)
    std::atomic<uint64_t> droppedUnverified{ 0 }; //!< payload failed the verify gate
    std::atomic<uint64_t> droppedStale{ 0 }; //!< seq regression / duplicate
    std::atomic<uint64_t> droppedTruncated{ 0 }; //!< datagram larger than the recv buffer (MSG_TRUNC)
    std::atomic<uint64_t> warnedNonWholeMs{ 0 }; //!< sampleTime not a whole millisecond
    std::atomic<uint64_t> warnedTimeBackward{ 0 }; //!< sampleTime went backward
};

//! Single-threaded, blocking UDP receive loop. One datagram = one frame. Not copyable (owns a
//! socket fd). See the per-datagram pipeline in processDatagram().
class TeleopUdpReceiver
{
public:
    explicit TeleopUdpReceiver(TeleopLogFn logFn = TeleopLogFn());
    ~TeleopUdpReceiver();

    TeleopUdpReceiver(const TeleopUdpReceiver&) = delete;
    TeleopUdpReceiver& operator=(const TeleopUdpReceiver&) = delete;

    //! Bind 0.0.0.0:port with SO_REUSEADDR and a 100 ms SO_RCVTIMEO so run() polls the stop flag.
    //! Idempotent: closes any prior socket first. \return false on any socket/bind error (logged).
    bool open(uint16_t port);

    //! Blocking receive loop on THIS thread: runs the per-datagram pipeline and invokes \c sink
    //! for each delivered frame. Returns when \c stop becomes true or on a fatal socket error.
    //! The TeleopFrame passed to \c sink (and its payload) is valid only until the sink returns.
    void run(const TeleopFrameSink& sink, const std::atomic<bool>& stop);

    //! Close the socket. Safe to call repeatedly; called by the destructor.
    void close();

    const TeleopReceiverStats& stats() const;

    //! Test seam: the full per-datagram pipeline minus the socket. Framing -> verify gate ->
    //! (seq state machine) -> sink. Public so the malformed / seq case tables can be driven
    //! without opening a socket. \c data must not be null unless \c size is 0.
    void processDatagram(const uint8_t* data, size_t size, const TeleopFrameSink& sink);

private:
    //! Per-category log throttle. A malformed/lossy flood must not DoS the log, so each category
    //! logs only its 1st occurrence and every 100th after that.
    enum LogCategory
    {
        LC_Malformed = 0,
        LC_Unverified,
        LC_Stale,
        LC_Gap,
        LC_Reset,
        LC_Truncated,
        LC_NonWholeMs,
        LC_TimeBackward,
        LC_Count
    };

    void logMessage(const char* message);
    //! Emit \c message for \c category only on its 1st and every 100th occurrence. Single-writer:
    //! called only from the receive thread (processDatagram / run), so the counters are plain.
    void logRateLimited(LogCategory category, const char* message);

    TeleopLogFn m_logFn;
    int m_socket = -1;
    bool m_haveSession = false;
    uint64_t m_lastSeq = 0;
    int64_t m_lastSampleTimeNs = 0;
    TeleopReceiverStats m_stats;
    //! Occurrences seen per LogCategory, for rate limiting. Receive-thread-only (plain integers).
    uint64_t m_logCounts[LC_Count] = {};
    //! Larger than the maximum IPv4 UDP payload (65507) so a valid datagram is never truncated.
    std::array<uint8_t, 65536> m_recvBuffer;
};
}

#endif
