import Foundation

struct Config {
    var user = "revine"
    /// known_hosts has the Pi's key under this name (its first DHCP address).
    var hostKeyAlias = "192.168.8.242"
    /// ED25519 key of the car's Pi: an address only counts as the Pi if it has this key.
    var fingerprint = "SHA256:NEHaEUFIi9CALGWtaSNaSTT46r0xAvXM1qDEDUWsrGg"
    var candidates = ["192.168.8.153", "192.168.8.242"]
    var scanSubnets = ["192.168.8"]
    var interval: TimeInterval = 2
    var rosInterval: TimeInterval = 30
    var lowBatteryVolts = 6.8

    /// Overrides from `defaults write PiMonitor <key> <value>` or the command line
    /// (`PiMonitor -user pi -hostKeyAlias 10.0.0.5 -fingerprint SHA256:...`), for another car.
    init() {
        let d = UserDefaults.standard
        if let v = d.string(forKey: "user") { user = v }
        if let v = d.string(forKey: "hostKeyAlias") { hostKeyAlias = v }
        if let v = d.string(forKey: "fingerprint") { fingerprint = v }
        if let v = d.string(forKey: "subnets") { scanSubnets = v.split(separator: ",").map(String.init) }
    }
}

struct CPUTimes {
    var busy: UInt64
    var total: UInt64
}

struct ThrottleFlags {
    var raw: UInt32
    static let names: [(bit: UInt32, now: String, past: String)] = [
        (0, "Bajo voltaje AHORA", "Hubo bajo voltaje"),
        (1, "Frecuencia limitada", "Hubo frecuencia limitada"),
        (2, "Estrangulado AHORA", "Hubo estrangulamiento"),
        (3, "Límite térmico suave", "Hubo límite térmico"),
    ]
    var now: [String] { Self.names.filter { raw & (1 << $0.bit) != 0 }.map(\.now) }
    var past: [String] { Self.names.filter { raw & (1 << ($0.bit + 16)) != 0 }.map(\.past) }
    var underVoltageNow: Bool { raw & 1 != 0 }
    var anyNow: Bool { raw & 0xF != 0 }
}

struct RosProcess: Hashable {
    var name: String
    var seconds: Int
}

/// One fast probe of the Pi.
struct PiSample {
    var date = Date()
    var hostname = ""
    var bootID = ""
    var uptime: TimeInterval = 0
    var load: [Double] = []
    var cpu: [CPUTimes] = []          // index 0 = all cores, then cpu0, cpu1...
    var tempC: Double?
    var freqMHz: [Double] = []
    var memTotalKB: Double = 0, memAvailKB: Double = 0, swapTotalKB: Double = 0, swapFreeKB: Double = 0
    var diskTotalKB: Double = 0, diskUsedKB: Double = 0
    var throttled: ThrottleFlags?
    var gpuTempC: Double?
    var clocksMHz: [String: Double] = [:]
    var coreVolts: Double?
    var ext5V: Double?
    var wifiIface: String?
    var wifiLinkQuality: Double?
    var wifiSignal: Double?
    var ssid: String?
    var rxBitrate: String?
    var txBitrate: String?
    var wifiFreq: String?
    var ips: [String] = []
    var ros: [RosProcess] = []
    var complete = false

    static func parse(_ text: String) -> PiSample {
        var s = PiSample()
        for raw in text.split(separator: "\n") {
            let line = String(raw)
            guard let space = line.firstIndex(of: " ") else { if line == "END" { s.complete = true }; continue }
            let key = line[..<space]
            let rest = line[line.index(after: space)...].trimmingCharacters(in: .whitespaces)
            let f = rest.split(separator: " ").map(String.init)
            switch key {
            case "HOST": s.hostname = rest
            case "BOOT": s.bootID = rest
            case "UP": s.uptime = Double(rest) ?? 0
            case "LOAD": s.load = f.prefix(3).compactMap(Double.init)
            case "STAT":
                let v = f.dropFirst().compactMap(UInt64.init)
                if v.count >= 5 {
                    let idle = v[3] + v[4]                       // idle + iowait
                    let total = v.prefix(8).reduce(0, +)         // without guest (already in user)
                    s.cpu.append(CPUTimes(busy: total - idle, total: total))
                }
            case "TEMP": s.tempC = Double(rest).map { $0 / 1000 }
            case "FREQ": if let k = Double(rest) { s.freqMHz.append(k / 1000) }
            case "MEM":
                let v = Double(f.last ?? "") ?? 0
                switch f.first {
                case "MemTotal:": s.memTotalKB = v
                case "MemAvailable:": s.memAvailKB = v
                case "SwapTotal:": s.swapTotalKB = v
                case "SwapFree:": s.swapFreeKB = v
                default: break
                }
            case "DISK":
                s.diskTotalKB = Double(f.first ?? "") ?? 0
                s.diskUsedKB = f.count > 1 ? Double(f[1]) ?? 0 : 0
            case "THR":   // throttled=0x50000
                if let hex = rest.split(separator: "=").last, hex.hasPrefix("0x") {
                    s.throttled = UInt32(hex.dropFirst(2), radix: 16).map(ThrottleFlags.init)
                }
            case "VTEMP": s.gpuTempC = number(after: "=", in: rest)          // temp=52.1'C
            case "CLK":   // CLK arm frequency(0)=2400000000
                if let name = f.first, let hz = number(after: "=", in: rest) { s.clocksMHz[name] = hz / 1e6 }
            case "VOLT": s.coreVolts = number(after: "=", in: rest)          // volt=0.7200V
            case "EXT5V": s.ext5V = number(after: "=", in: rest)             // EXT5V_V volt(24)=5.1234V
            case "WIRELESS":  // iface quality. level.
                if f.count >= 3 {
                    s.wifiIface = f[0]
                    s.wifiLinkQuality = Double(f[1].trimmingCharacters(in: CharacterSet(charactersIn: ".")))
                    s.wifiSignal = Double(f[2].trimmingCharacters(in: CharacterSet(charactersIn: ".")))
                }
            case "IW":
                if rest.hasPrefix("SSID:") { s.ssid = String(rest.dropFirst(5)).trimmingCharacters(in: .whitespaces) }
                else if rest.hasPrefix("signal:") { s.wifiSignal = Double(f.dropFirst().first ?? "") ?? s.wifiSignal }
                else if rest.hasPrefix("rx bitrate:") { s.rxBitrate = bitrate(rest) }
                else if rest.hasPrefix("tx bitrate:") { s.txBitrate = bitrate(rest) }
                else if rest.hasPrefix("freq:") { s.wifiFreq = f.dropFirst().first }
            case "IP": s.ips = f.filter { $0.contains(".") }
            case "PROC":
                if let secs = Int(f.first ?? ""), let name = rosName(Array(f.dropFirst())) {
                    s.ros.append(RosProcess(name: name, seconds: secs))
                }
            default: break
            }
        }
        // Several processes can map to the same name (python wrapper + node): keep one each.
        var seen = Set<String>()
        s.ros = s.ros.filter { seen.insert($0.name).inserted }
        return s
    }

    private static func number(after sep: Character, in text: String) -> Double? {
        guard let part = text.split(separator: sep).last else { return nil }
        let digits = part.prefix { "0123456789.-".contains($0) }
        return Double(digits)
    }

    private static func bitrate(_ text: String) -> String {
        // "rx bitrate: 72.2 MBit/s MCS 7 short GI" -> "72.2 MBit/s"
        let f = text.split(separator: " ")
        return f.count >= 4 ? "\(f[2]) \(f[3])" : text
    }

    /// Short name for a ROS process: the __node:= name, the launch file, or the executable.
    static func rosName(_ args: [String]) -> String? {
        if let n = args.first(where: { $0.hasPrefix("__node:=") }) { return String(n.dropFirst(8)) }
        if let i = args.firstIndex(where: { $0.hasSuffix("ros2") }), args.count > i + 1, args[i + 1] == "launch" {
            return "launch " + args.dropFirst(i + 2).prefix(2).joined(separator: " ")
        }
        if args.contains("--ros-args") {
            let exe = args.first(where: { !$0.hasPrefix("/usr/bin/python") && $0 != "python3" }) ?? args.first ?? "?"
            return URL(fileURLWithPath: exe).lastPathComponent
        }
        return nil
    }

    var memUsedFraction: Double { memTotalKB > 0 ? 1 - memAvailKB / memTotalKB : 0 }
    var diskUsedFraction: Double { diskTotalKB > 0 ? diskUsedKB / diskTotalKB : 0 }
}

/// Result of the (slower) ROS probe.
struct RosSample: Decodable {
    var nodes: [String]
    var rates: [String: Double?]
    var battery_mv: Int?
    var topics: Int
}
