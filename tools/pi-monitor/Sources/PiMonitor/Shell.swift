import Foundation

struct CommandResult {
    var status: Int32
    var stdout: String
    var stderr: String
    var timedOut: Bool
    var ok: Bool { status == 0 && !timedOut }
}

/// Runs a program off the main thread with a timeout; `input` is written to its stdin.
func run(_ path: String, _ args: [String], input: String? = nil, timeout: TimeInterval) async -> CommandResult {
    await withCheckedContinuation { continuation in
        DispatchQueue.global(qos: .utility).async {
            let process = Process()
            process.executableURL = URL(fileURLWithPath: path)
            process.arguments = args
            let out = Pipe(), err = Pipe(), inp = Pipe()
            process.standardOutput = out
            process.standardError = err
            process.standardInput = inp
            var outData = Data(), errData = Data()
            let group = DispatchGroup()
            group.enter(); group.enter()
            DispatchQueue.global().async { outData = out.fileHandleForReading.readDataToEndOfFile(); group.leave() }
            DispatchQueue.global().async { errData = err.fileHandleForReading.readDataToEndOfFile(); group.leave() }
            do {
                try process.run()
            } catch {
                continuation.resume(returning: CommandResult(status: -1, stdout: "", stderr: "\(error)", timedOut: false))
                return
            }
            if let input { inp.fileHandleForWriting.write(Data(input.utf8)) }
            try? inp.fileHandleForWriting.close()
            var timedOut = false
            let deadline = DispatchTime.now() + timeout
            if group.wait(timeout: deadline) == .timedOut {
                timedOut = true
                process.terminate()
                _ = group.wait(timeout: .now() + 2)
            }
            process.waitUntilExit()
            continuation.resume(returning: CommandResult(
                status: process.terminationStatus,
                stdout: String(decoding: outData, as: UTF8.self),
                stderr: String(decoding: errData, as: UTF8.self),
                timedOut: timedOut))
        }
    }
}

enum SSH {
    /// Common options. The ControlMaster keeps one TCP/SSH session open, so each 2 s probe is a
    /// new channel on it (no new handshake: almost no CPU on the Pi). ServerAlive drops a dead
    /// master in ~6 s so the app sees Wi-Fi loss quickly.
    static func options(_ config: Config) -> [String] {
        [
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=3",
            "-o", "HostKeyAlias=\(config.hostKeyAlias)",
            "-o", "StrictHostKeyChecking=yes",
            "-o", "ControlMaster=auto",
            "-o", "ControlPath=/tmp/pimonitor-%C",
            "-o", "ControlPersist=60",
            "-o", "ServerAliveInterval=2",
            "-o", "ServerAliveCountMax=3",
            "-o", "LogLevel=ERROR",
        ]
    }

    static func run(_ config: Config, host: String, command: String, input: String, timeout: TimeInterval) async -> CommandResult {
        await PiMonitor.run("/usr/bin/ssh", options(config) + ["\(config.user)@\(host)", command],
                            input: input, timeout: timeout)
    }

    /// SHA256 fingerprint of the host's ED25519 key, without logging in.
    static func fingerprint(host: String) async -> String? {
        let scan = await PiMonitor.run("/usr/bin/ssh-keyscan", ["-T", "2", "-t", "ed25519", host], timeout: 4)
        guard scan.ok, !scan.stdout.isEmpty else { return nil }
        let fp = await PiMonitor.run("/usr/bin/ssh-keygen", ["-lf", "-"], input: scan.stdout, timeout: 3)
        return fp.stdout.split(separator: " ").first(where: { $0.hasPrefix("SHA256:") }).map(String.init)
    }
}

/// Round trip from the Mac in ms (nil = lost).
func ping(_ host: String) async -> Double? {
    let r = await run("/sbin/ping", ["-c", "1", "-t", "1", "-q", host], timeout: 2.5)
    // round-trip min/avg/max/stddev = 4.1/4.1/4.1/0.0 ms
    guard r.ok, let line = r.stdout.split(separator: "\n").first(where: { $0.contains("min/avg/max") }),
          let values = line.split(separator: "=").last?.trimmingCharacters(in: .whitespaces).split(separator: "/"),
          values.count >= 2 else { return nil }
    return Double(values[1])
}
