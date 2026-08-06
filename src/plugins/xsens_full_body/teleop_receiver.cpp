// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Embedded from mvn_isaac_devtools/tools/teleop_receiver/. Reformatted to
// the fork's clang-format-14 style on embed (SortIncludes + reflow); the receiver LOGIC is mirrored,
// not rewritten. See teleop_receiver.h for the one deliberate difference on this branch.

#include "teleop_receiver.h"

#include "picofullbody_converter.h"
#include "teleop_wire.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unistd.h>
#include <utility>

namespace teleop
{
TeleopUdpReceiver::TeleopUdpReceiver(TeleopLogFn logFn) : m_logFn(std::move(logFn))
{
}

TeleopUdpReceiver::~TeleopUdpReceiver()
{
    close();
}

void TeleopUdpReceiver::logMessage(const char* message)
{
    if (m_logFn)
        m_logFn(message);
    else
        std::fprintf(stderr, "%s\n", message);
}

void TeleopUdpReceiver::logRateLimited(LogCategory category, const char* message)
{
    const uint64_t n = ++m_logCounts[category];
    if (n == 1 || (n % 100) == 0)
        logMessage(message);
}

bool TeleopUdpReceiver::open(uint16_t port)
{
    close(); // idempotent re-open

    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        logMessage("teleop receiver: socket() failed");
        return false;
    }

    int reuse = 1;
    if (::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        logMessage("teleop receiver: SO_REUSEADDR failed");
        ::close(sock);
        return false;
    }

    // 100 ms recv timeout: recvfrom returns EAGAIN on idle so run() can re-check the stop flag.
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100 * 1000;
    if (::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        logMessage("teleop receiver: SO_RCVTIMEO failed");
        ::close(sock);
        return false;
    }

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        logMessage("teleop receiver: bind() failed");
        ::close(sock);
        return false;
    }

    m_socket = sock;
    m_port = port;
    return true;
}

void TeleopUdpReceiver::injectRecvErrors(int errnoValue, int times)
{
    m_injectRecvErrno = errnoValue;
    m_injectRecvErrorsLeft = times;
}

bool TeleopUdpReceiver::recoverSocket(const std::atomic<bool>& stop)
{
    // Without a port, recovery would bind an ephemeral one the sender cannot reach.
    if (m_port == 0)
    {
        m_stats.socketRecoveryFailures.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_RecoveryFailed, "teleop receiver: socket error before a successful open(), stopping");
        return false;
    }

    static constexpr int backoffMs[MaxRecoveryAttempts] = { 100, 200, 400, 800, 800 }; // ~2.3 s total
    static constexpr int sliceMs = 50;

    for (int attempt = 0; attempt < MaxRecoveryAttempts; ++attempt)
    {
        // Sliced so a stop request does not wait out the whole backoff.
        for (int remaining = backoffMs[attempt]; remaining > 0; remaining -= sliceMs)
        {
            if (stop.load(std::memory_order_relaxed))
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(remaining < sliceMs ? remaining : sliceMs));
        }

        if (stop.load(std::memory_order_relaxed))
            return false;

        if (open(m_port)) // idempotent: closes the dead socket, then re-binds
        {
            m_stats.socketRecoveries.fetch_add(1, std::memory_order_relaxed);
            logRateLimited(LC_Recovered, "teleop receiver: socket recovered, receive loop continuing");
            return true;
        }
    }

    m_stats.socketRecoveryFailures.fetch_add(1, std::memory_order_relaxed);
    logRateLimited(LC_RecoveryFailed, "teleop receiver: socket recovery exhausted its retries, stopping");
    return false;
}

void TeleopUdpReceiver::close()
{
    if (m_socket >= 0)
    {
        ::close(m_socket);
        m_socket = -1;
    }
}

const TeleopReceiverStats& TeleopUdpReceiver::stats() const
{
    return m_stats;
}

void TeleopUdpReceiver::run(const TeleopFrameSink& sink, const std::atomic<bool>& stop)
{
    while (!stop.load(std::memory_order_relaxed))
    {
        sockaddr_in src;
        socklen_t srcLen = sizeof(src);
        ssize_t n;
        if (m_injectRecvErrorsLeft > 0) // test seam, see injectRecvErrors()
        {
            --m_injectRecvErrorsLeft;
            errno = m_injectRecvErrno;
            n = -1;
        }
        else
        {
            n = ::recvfrom(m_socket, m_recvBuffer.data(), m_recvBuffer.size(), MSG_TRUNC,
                           reinterpret_cast<sockaddr*>(&src), &srcLen);
        }

        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue; // recv timeout / interrupt: re-check stop and keep going

            if (!recoverSocket(stop))
                break; // stop requested, or the retry budget ran out: the only non-stop exit
            continue;
        }

        if (static_cast<size_t>(n) > m_recvBuffer.size())
        {
            // With MSG_TRUNC, recvfrom reports the full datagram length even when it overflowed
            // the buffer. Unreachable for IPv4 UDP (buffer > 65507) but counted as belt-and-braces
            // so an over-large datagram is dropped rather than processed from a partial buffer.
            m_stats.droppedTruncated.fetch_add(1, std::memory_order_relaxed);
            logRateLimited(LC_Truncated, "teleop receiver: dropped over-large datagram");
            continue;
        }

        processDatagram(m_recvBuffer.data(), static_cast<size_t>(n), sink);
    }
}

void TeleopUdpReceiver::processDatagram(const uint8_t* data, size_t size, const TeleopFrameSink& sink)
{
    // 1) Framing: header parse + payload bounds. Validates NO payload content.
    picofullbody::TeleopFrameView view;
    if (!picofullbody::deserializeTeleopFrame(data, size, view))
    {
        m_stats.droppedMalformed.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_Malformed, "teleop receiver: dropped malformed datagram");
        return;
    }

    // 2) SAFETY GATE: structural verify of the payload BEFORE anything downstream can decode it,
    //    and before any session-state change, so a hostile payload can never advance the stream.
    if (!picofullbody::verifyFullBodyPosePicoPayload(view.payload, view.payloadLen))
    {
        m_stats.droppedUnverified.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_Unverified, "teleop receiver: dropped unverified payload");
        return;
    }

    // 3) Seq state machine (§4.3): decide deliver-vs-drop and whether this frame opens a session.
    //    Runs AFTER the verify gate so a malformed datagram can never advance stream state, and
    //    only a delivered frame commits the new seq/time below. Never throws, never asserts.
    const uint64_t seq = view.header.seq;
    bool sessionStart = false;
    if (!m_haveSession)
    {
        // Mid-stream attach is supported: the first verified frame opens a session at any seq.
        sessionStart = true;
    }
    else if (seq == m_lastSeq + 1)
    {
        // Contiguous: the common case, nothing to record.
    }
    else if (seq == 0 && m_lastSeq != 0)
    {
        // Sequence reset -> a new session (recording restarted). seq 0 with a zero last seq is a
        // duplicate, not a reset, and falls through to the stale branch below.
        m_stats.resets.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_Reset, "teleop receiver: sequence reset -> new session");
        sessionStart = true;
    }
    else if (seq > m_lastSeq + 1)
    {
        // Gap: datagrams lost in transit. Count the missing span, warn, still deliver.
        m_stats.gapsTotal.fetch_add(seq - m_lastSeq - 1, std::memory_order_relaxed);
        logRateLimited(LC_Gap, "teleop receiver: sequence gap (frames lost)");
    }
    else
    {
        // seq <= m_lastSeq: regression or duplicate (incl. seq == m_lastSeq == 0). A stale pose
        // delivered after a newer one would jerk the robot backward, so drop it; state unchanged.
        m_stats.droppedStale.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_Stale, "teleop receiver: dropped stale/duplicate frame");
        return;
    }

    // 4) Timestamp diagnostics (warn-only; the frame is delivered regardless). A session boundary
    //    legitimately restarts the clock, so the backward check is suppressed on sessionStart.
    const int64_t sampleTimeNs = view.header.sampleTimeNs;
    if (sampleTimeNs % 1000000LL != 0)
    {
        m_stats.warnedNonWholeMs.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_NonWholeMs, "teleop receiver: sample time is not a whole millisecond");
    }
    if (!sessionStart && sampleTimeNs < m_lastSampleTimeNs)
    {
        m_stats.warnedTimeBackward.fetch_add(1, std::memory_order_relaxed);
        logRateLimited(LC_TimeBackward, "teleop receiver: sample time went backward");
    }

    // Deliver, then commit stream state — only delivered frames advance seq/time.
    if (sink)
    {
        sink(TeleopFrame{ view.payload, view.payloadLen, seq, sampleTimeNs, view.header.rawDeviceTimeNs, sessionStart });
    }
    m_stats.delivered.fetch_add(1, std::memory_order_relaxed);
    m_haveSession = true;
    m_lastSeq = seq;
    m_lastSampleTimeNs = sampleTimeNs;
}
}
