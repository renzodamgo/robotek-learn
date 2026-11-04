import Darwin
import Foundation

/// Finds the Pi when its DHCP address changed: tries port 22 on every address of the given /24
/// subnets at once (one non-blocking connect each, a single poll), then keeps the hosts whose
/// ED25519 key has the Pi's fingerprint.
enum Discovery {
    /// /24 prefixes ("192.168.3") of the Mac's own IPv4 interfaces (Wi-Fi, Ethernet).
    static func localSubnets() -> [String] {
        var result: [String] = []
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&ifaddr) == 0, let first = ifaddr else { return [] }
        defer { freeifaddrs(ifaddr) }
        for ptr in sequence(first: first, next: { $0.pointee.ifa_next }) {
            let ifa = ptr.pointee
            guard let addr = ifa.ifa_addr, addr.pointee.sa_family == UInt8(AF_INET),
                  String(cString: ifa.ifa_name).hasPrefix("en") else { continue }
            var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            getnameinfo(addr, socklen_t(addr.pointee.sa_len), &host, socklen_t(host.count), nil, 0, NI_NUMERICHOST)
            let ip = String(cString: host)
            let parts = ip.split(separator: ".")
            if parts.count == 4, !ip.hasPrefix("169.254") {
                result.append(parts.prefix(3).joined(separator: "."))
            }
        }
        return result
    }

    /// Hosts of `subnets` with TCP port 22 open.
    static func openSSH(subnets: [String], timeoutMs: Int32 = 900) -> [String] {
        var fds: [(Int32, String)] = []
        defer { for (fd, _) in fds { close(fd) } }
        for subnet in Set(subnets) {
            for i in 1...254 {
                let ip = "\(subnet).\(i)"
                let fd = socket(AF_INET, SOCK_STREAM, 0)
                guard fd >= 0 else { continue }
                _ = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK)
                var addr = sockaddr_in()
                addr.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
                addr.sin_family = sa_family_t(AF_INET)
                addr.sin_port = in_port_t(22).bigEndian
                inet_pton(AF_INET, ip, &addr.sin_addr)
                let rc = withUnsafePointer(to: &addr) {
                    $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                        connect(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size))
                    }
                }
                if rc == 0 || errno == EINPROGRESS { fds.append((fd, ip)) } else { close(fd) }
            }
        }
        var open: [String] = []
        var pending = fds
        let deadline = Date().addingTimeInterval(Double(timeoutMs) / 1000)
        while !pending.isEmpty, Date() < deadline {
            var pfds = pending.map { pollfd(fd: $0.0, events: Int16(POLLOUT), revents: 0) }
            let left = Int32(max(1, deadline.timeIntervalSinceNow * 1000))
            guard poll(&pfds, nfds_t(pfds.count), left) > 0 else { break }
            var still: [(Int32, String)] = []
            for (i, p) in pfds.enumerated() {
                if p.revents == 0 { still.append(pending[i]); continue }
                var err: Int32 = 0
                var len = socklen_t(MemoryLayout<Int32>.size)
                getsockopt(p.fd, SOL_SOCKET, SO_ERROR, &err, &len)
                if err == 0 && p.revents & Int16(POLLOUT) != 0 { open.append(pending[i].1) }
            }
            pending = still
        }
        return open
    }

    /// First host (candidates first, then a scan) whose host key is the Pi's.
    static func find(_ config: Config, lastKnown: String?) async -> String? {
        var subnets = config.scanSubnets + localSubnets()
        if let lastKnown { subnets.insert(lastKnown.split(separator: ".").prefix(3).joined(separator: "."), at: 0) }
        let open = await Task.detached { openSSH(subnets: subnets) }.value
        let preferred = ([lastKnown].compactMap { $0 } + config.candidates).filter(open.contains)
        for host in preferred + open.filter({ !preferred.contains($0) }) {
            if await SSH.fingerprint(host: host) == config.fingerprint { return host }
        }
        return nil
    }
}
