import Foundation
import SwiftUI

enum LinkState: String {
    case connecting = "Conectando"
    case online = "En línea"
    case offline = "Sin conexión"
    case searching = "Buscando el Pi"
}

struct HistoryPoint: Identifiable {
    let id = UUID()
    var date: Date
    var online: Bool
    var cpu: Double?
    var tempC: Double?
    var pingMs: Double?
    var signal: Double?
    var batteryV: Double?
    var ext5V: Double?
}

struct PiEvent: Identifiable, Codable {
    var id = UUID()
    var date: Date
    var kind: String       // offline, online, reboot, throttle, ip, ros, battery, wifi
    var text: String
}

@MainActor
final class Monitor: ObservableObject {
    let config = Config()
    @Published var state: LinkState = .connecting
    @Published var host: String?
    @Published var sample: PiSample?
    @Published var coreUsage: [Double] = []      // 0...1, index 0 = all cores
    @Published var pingMs: Double?
    @Published var ros: RosSample?
    @Published var rosDate: Date?
    @Published var rosError: String?
    @Published var batteryV: Double?
    @Published var lastSeen: Date?
    @Published var lastError: String?
    @Published var history: [HistoryPoint] = []
    @Published var events: [PiEvent] = []

    private var previous: PiSample?
    private var failures = 0
    private var lastScan = Date.distantPast
    private var rosTask: Task<Void, Never>?
    private var lastRosRun = Date.distantPast
    private var pastThrottle: UInt32 = 0
    private var batteryLowLogged = false
    private let historyLimit = 3600               // 2 h at 2 s
    private let folder: URL
    private var loop: Task<Void, Never>?

    init() {
        folder = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("PiMonitor", isDirectory: true)
        try? FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
        host = UserDefaults.standard.string(forKey: "host") ?? config.candidates.first
        loadEvents()
        loop = Task { await self.run() }
    }

    var logFolder: URL { folder }

    // MARK: loop

    private func run() async {
        while !Task.isCancelled {
            let start = Date()
            await tick()
            let wait = config.interval - Date().timeIntervalSince(start)
            if wait > 0 { try? await Task.sleep(for: .seconds(wait)) }
        }
    }

    func scanNow() {
        lastScan = .distantPast
        failures = max(failures, 3)
    }

    private func tick() async {
        if failures >= 3, Date().timeIntervalSince(lastScan) > 20 {
            await search()
        }
        guard let host else { record(online: false); return }
        async let pingResult = ping(host)
        let result = await SSH.run(config, host: host, command: "sh -s", input: Scripts.fast, timeout: 8)
        pingMs = await pingResult
        let s = PiSample.parse(result.stdout)
        if result.ok, s.complete {
            handle(s)
        } else {
            failures += 1
            lastError = result.timedOut ? "ssh: tiempo agotado" :
                result.stderr.trimmingCharacters(in: .whitespacesAndNewlines).split(separator: "\n").last.map(String.init)
            if failures == 2, state == .online || state == .connecting {
                let since = lastSeen.map { " (último dato \($0.formatted(date: .omitted, time: .standard)))" } ?? ""
                log("offline", "Sin conexión\(since)" + (pingMs == nil ? ", tampoco responde a ping" : ", el ping sí responde"))
            }
            if failures >= 2 { state = failures >= 3 ? .searching : .offline }
            record(online: false)
        }
    }

    private func search() async {
        lastScan = Date()
        state = .searching
        if let found = await Discovery.find(config, lastKnown: host) {
            if found != host {
                log("ip", "El Pi está ahora en \(found)" + (host.map { " (antes \($0))" } ?? ""))
                host = found
                UserDefaults.standard.set(found, forKey: "host")
            }
            failures = 2          // try it on this tick
        } else {
            state = .offline
        }
    }

    private func handle(_ s: PiSample) {
        if state != .online {
            let down = lastSeen.map { " tras \(Self.duration(Date().timeIntervalSince($0))) sin datos" } ?? ""
            if !events.isEmpty || lastSeen != nil { log("online", "En línea en \(host ?? "?")\(down)") }
        }
        if let prev = previous {
            if prev.bootID != s.bootID {
                log("reboot", "El Pi se reinició: arrancó hace \(Self.duration(s.uptime)) (\(Date().addingTimeInterval(-s.uptime).formatted(date: .omitted, time: .standard)))")
                pastThrottle = 0
            } else {
                coreUsage = zip(prev.cpu, s.cpu).map { a, b in
                    let total = Double(b.total &- a.total)
                    return total > 0 ? min(1, max(0, Double(b.busy &- a.busy) / total)) : 0
                }
            }
            if prev.ros.isEmpty != s.ros.isEmpty {
                log("ros", s.ros.isEmpty ? "El stack ROS se detuvo" : "Stack ROS en marcha: " + s.ros.map(\.name).joined(separator: ", "))
            }
            if let a = prev.wifiSignal, let b = s.wifiSignal, a > -75, b <= -75 {
                log("wifi", "Señal Wi-Fi débil: \(Int(b)) dBm")
            }
        } else if lastSeen == nil {
            log("online", "Conectado a \(s.hostname) en \(host ?? "?"), encendido hace \(Self.duration(s.uptime))")
        }
        if let t = s.throttled {
            let newPast = (t.raw >> 16) & ~pastThrottle & 0xF
            if newPast != 0 || (previous?.throttled?.anyNow == false && t.anyNow) {
                log("throttle", "get_throttled 0x\(String(t.raw, radix: 16)): " + (t.now + t.past).joined(separator: ", "))
            }
            pastThrottle |= (t.raw >> 16) & 0xF
        }
        previous = s
        sample = s
        failures = 0
        state = .online
        lastSeen = Date()
        lastError = nil
        if s.ros.isEmpty { ros = nil }
        maybeRunRos(s)
        record(online: true)
    }

    private func maybeRunRos(_ s: PiSample) {
        guard !s.ros.isEmpty, rosTask == nil, Date().timeIntervalSince(lastRosRun) >= config.rosInterval,
              let host else { return }
        lastRosRun = Date()
        rosTask = Task {
            // Interactive zsh so ~/.zshrc sources ROS and sets ROS_DOMAIN_ID like a normal login.
            let r = await SSH.run(config, host: host, command: "zsh -ic 'exec timeout 15 python3 -'",
                                  input: Scripts.ros, timeout: 20)
            if let line = r.stdout.split(separator: "\n").last(where: { $0.hasPrefix("ROSJSON ") }),
               let decoded = try? JSONDecoder().decode(RosSample.self, from: Data(line.dropFirst(8).utf8)) {
                ros = decoded
                rosDate = Date()
                rosError = nil
                if let mv = decoded.battery_mv {
                    batteryV = Double(mv) / 1000
                    if batteryV! < config.lowBatteryVolts, !batteryLowLogged {
                        log("battery", String(format: "Batería baja: %.2f V", batteryV!))
                        batteryLowLogged = true
                    } else if batteryV! >= config.lowBatteryVolts + 0.2 { batteryLowLogged = false }
                }
            } else {
                rosError = r.timedOut ? "La sonda ROS tardó demasiado" :
                    (r.stderr.split(separator: "\n").last.map(String.init) ?? "Sin respuesta de la sonda ROS")
            }
            rosTask = nil
        }
    }

    // MARK: history

    private func record(online: Bool) {
        let p = HistoryPoint(date: Date(), online: online,
                             cpu: online ? coreUsage.first : nil,
                             tempC: online ? sample?.tempC : nil,
                             pingMs: pingMs,
                             signal: online ? sample?.wifiSignal : nil,
                             batteryV: online ? batteryV : nil,
                             ext5V: online ? sample?.ext5V : nil)
        history.append(p)
        if history.count > historyLimit { history.removeFirst(history.count - historyLimit) }
        appendCSV(p)
    }

    private func appendCSV(_ p: HistoryPoint) {
        let day = p.date.formatted(.iso8601.year().month().day())
        let url = folder.appendingPathComponent("samples-\(day).csv")
        func f(_ v: Double?) -> String { v.map { String(format: "%.3f", $0) } ?? "" }
        let s = sample
        let line = [p.date.formatted(.iso8601), host ?? "", p.online ? "1" : "0", f(p.pingMs), f(p.cpu.map { $0 * 100 }),
                    f(p.tempC), f(p.batteryV), f(p.ext5V), f(p.signal),
                    p.online ? s?.throttled.map { "0x" + String($0.raw, radix: 16) } ?? "" : "",
                    p.online ? f(s?.uptime) : "", p.online ? "\(s?.ros.count ?? 0)" : ""].joined(separator: ",") + "\n"
        if !FileManager.default.fileExists(atPath: url.path) {
            try? "time,host,online,ping_ms,cpu_pct,temp_c,battery_v,ext5v_v,signal_dbm,throttled,uptime_s,ros_procs\n"
                .write(to: url, atomically: true, encoding: .utf8)
        }
        if let h = try? FileHandle(forWritingTo: url) {
            h.seekToEndOfFile(); h.write(Data(line.utf8)); try? h.close()
        }
    }

    private var eventsURL: URL { folder.appendingPathComponent("events.jsonl") }

    private func loadEvents() {
        guard let text = try? String(contentsOf: eventsURL, encoding: .utf8) else { return }
        let decoder = JSONDecoder(); decoder.dateDecodingStrategy = .iso8601
        events = text.split(separator: "\n").suffix(300).compactMap { try? decoder.decode(PiEvent.self, from: Data($0.utf8)) }
    }

    func log(_ kind: String, _ text: String) {
        let e = PiEvent(date: Date(), kind: kind, text: text)
        events.append(e)
        if events.count > 300 { events.removeFirst(events.count - 300) }
        let encoder = JSONEncoder(); encoder.dateEncodingStrategy = .iso8601
        guard let data = try? encoder.encode(e) else { return }
        if !FileManager.default.fileExists(atPath: eventsURL.path) { FileManager.default.createFile(atPath: eventsURL.path, contents: nil) }
        if let h = try? FileHandle(forWritingTo: eventsURL) {
            h.seekToEndOfFile(); h.write(data + Data("\n".utf8)); try? h.close()
        }
    }

    static func duration(_ seconds: TimeInterval) -> String {
        let s = Int(seconds)
        if s < 60 { return "\(s) s" }
        if s < 3600 { return "\(s / 60) min \(s % 60) s" }
        if s < 86400 { return "\(s / 3600) h \(s % 3600 / 60) min" }
        return "\(s / 86400) d \(s % 86400 / 3600) h"
    }
}
