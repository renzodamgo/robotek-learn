import AppKit
import Charts
import SwiftUI

@main
struct PiMonitorApp: App {
    @StateObject private var monitor = Monitor()

    init() { NSApplication.shared.setActivationPolicy(.accessory) }   // menu bar only, no Dock icon

    var body: some Scene {
        MenuBarExtra {
            PanelView().environmentObject(monitor)
        } label: {
            MenuLabel(monitor: monitor)
        }
        .menuBarExtraStyle(.window)

        // The same panel as a normal window (button "Abrir ventana", or `-openWindow YES` at launch).
        Window("Pi del coche", id: "panel") {
            PanelView().environmentObject(monitor)
        }
        .windowResizability(.contentSize)
    }
}

struct MenuLabel: View {
    @ObservedObject var monitor: Monitor
    @Environment(\.openWindow) private var openWindow
    var body: some View {
        content.onAppear {
            if UserDefaults.standard.bool(forKey: "openWindow") { openWindow(id: "panel") }
        }
    }
    @ViewBuilder private var content: some View {
        if monitor.state == .online, let s = monitor.sample {
            let warn = (s.throttled?.anyNow ?? false) || (monitor.batteryV ?? 99) < monitor.config.lowBatteryVolts
            Image(systemName: warn ? "exclamationmark.triangle.fill" : "cpu")
            Text(label(s))
        } else {
            Image(systemName: monitor.state == .searching ? "magnifyingglass" : "wifi.slash")
            Text("Pi")
        }
    }
    private func label(_ s: PiSample) -> String {
        var parts: [String] = []
        if let v = monitor.batteryV { parts.append(String(format: "%.2fV", v)) }
        if let t = s.tempC { parts.append(String(format: "%.0f°", t)) }
        return parts.isEmpty ? "Pi" : parts.joined(separator: " ")
    }
}

// MARK: panel

struct PanelView: View {
    @EnvironmentObject var m: Monitor
    @Environment(\.openWindow) private var openWindow

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 10) {
                header
                if let s = m.sample {
                    if m.state != .online {
                        Text("Datos de las \(m.lastSeen?.formatted(date: .omitted, time: .standard) ?? "?") (sin conexión ahora)")
                            .font(.caption).foregroundStyle(.orange)
                    }
                    power(s)
                    cpu(s)
                    memory(s)
                    wifi(s)
                    rosSection(s)
                }
                charts
                timeline
                eventList
                footer
            }
            .padding(12)
        }
        .frame(width: 440, height: 760)
    }

    private var header: some View {
        HStack(alignment: .firstTextBaseline) {
            Circle().fill(stateColor).frame(width: 10, height: 10)
            VStack(alignment: .leading) {
                Text("\(m.sample?.hostname ?? "Pi del coche") · \(m.state.rawValue)").font(.headline)
                Text([m.host, m.sample.map { "encendido " + Monitor.duration($0.uptime) },
                      m.pingMs.map { String(format: "ping %.0f ms", $0) } ?? "ping —"]
                    .compactMap { $0 }.joined(separator: " · "))
                    .font(.caption).foregroundStyle(.secondary)
                if let e = m.lastError, m.state != .online { Text(e).font(.caption2).foregroundStyle(.red).lineLimit(2) }
            }
            Spacer()
        }
    }

    private var stateColor: Color {
        switch m.state {
        case .online: return (m.sample?.throttled?.anyNow ?? false) ? .orange : .green
        case .connecting: return .gray
        case .offline: return .red
        case .searching: return .yellow
        }
    }

    private func power(_ s: PiSample) -> some View {
        GroupBox("Energía") {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    metric("Batería (RRC Lite)", m.batteryV.map { String(format: "%.2f V", $0) } ?? "—",
                           color: batteryColor)
                    metric("Entrada 5 V del Pi", s.ext5V.map { String(format: "%.2f V", $0) } ?? "—",
                           color: (s.ext5V ?? 5) < 4.85 ? .red : .primary)
                    metric("Núcleo", s.coreVolts.map { String(format: "%.3f V", $0) } ?? "—")
                }
                if m.batteryV == nil {
                    Text(m.sample?.ros.isEmpty ?? true ? "La batería se lee de /rrc/battery: hace falta el driver de la placa en marcha"
                         : "Leyendo /rrc/battery…").font(.caption2).foregroundStyle(.secondary)
                } else if let d = m.rosDate {
                    Text("Batería leída a las \(d.formatted(date: .omitted, time: .standard)) (cada \(Int(m.config.rosInterval)) s)")
                        .font(.caption2).foregroundStyle(.secondary)
                }
                if let t = s.throttled {
                    let flags = t.now.map { ($0, Color.red) } + t.past.map { ($0, Color.orange) }
                    HStack {
                        Text("get_throttled 0x\(String(t.raw, radix: 16))").font(.caption.monospaced())
                        if flags.isEmpty { Text("sin avisos").font(.caption).foregroundStyle(.green) }
                    }
                    ForEach(flags, id: \.0) { f in
                        Label(f.0, systemImage: "bolt.trianglebadge.exclamationmark").font(.caption).foregroundStyle(f.1)
                    }
                }
            }.frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    private var batteryColor: Color {
        guard let v = m.batteryV else { return .primary }
        return v < m.config.lowBatteryVolts ? .red : v < 7.2 ? .orange : .green
    }

    private func cpu(_ s: PiSample) -> some View {
        GroupBox("CPU y GPU") {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    metric("CPU", s.tempC.map { String(format: "%.1f °C", $0) } ?? "—",
                           color: (s.tempC ?? 0) > 80 ? .red : (s.tempC ?? 0) > 70 ? .orange : .primary)
                    metric("Carga", s.load.map { String(format: "%.2f", $0) }.joined(separator: " "))
                    metric("Total", m.coreUsage.first.map { String(format: "%.0f %%", $0 * 100) } ?? "—")
                }
                ForEach(Array(m.coreUsage.dropFirst().enumerated()), id: \.offset) { i, u in
                    HStack {
                        Text("cpu\(i)").font(.caption.monospaced()).frame(width: 34, alignment: .leading)
                        ProgressView(value: u).tint(u > 0.9 ? .red : u > 0.7 ? .orange : .accentColor)
                        Text(String(format: "%3.0f %%", u * 100)).font(.caption.monospaced()).frame(width: 40, alignment: .trailing)
                        Text(i < s.freqMHz.count ? String(format: "%4.0f MHz", s.freqMHz[i]) : "")
                            .font(.caption.monospaced()).foregroundStyle(.secondary).frame(width: 70, alignment: .trailing)
                    }
                }
                HStack {
                    metric("GPU (VideoCore)", s.gpuTempC.map { String(format: "%.1f °C", $0) } ?? "—")
                    metric("arm", s.clocksMHz["arm"].map { String(format: "%.0f MHz", $0) } ?? "—")
                    metric("core", s.clocksMHz["core"].map { String(format: "%.0f MHz", $0) } ?? "—")
                    metric("v3d", s.clocksMHz["v3d"].map { String(format: "%.0f MHz", $0) } ?? "—")
                }
            }
        }
    }

    private func memory(_ s: PiSample) -> some View {
        GroupBox("Memoria y disco") {
            VStack(spacing: 4) {
                bar("RAM", s.memUsedFraction, String(format: "%.1f / %.1f GB", (s.memTotalKB - s.memAvailKB) / 1048576, s.memTotalKB / 1048576))
                if s.swapTotalKB > 0 {
                    bar("Swap", 1 - s.swapFreeKB / s.swapTotalKB, String(format: "%.1f / %.1f GB", (s.swapTotalKB - s.swapFreeKB) / 1048576, s.swapTotalKB / 1048576))
                }
                bar("Disco /", s.diskUsedFraction, String(format: "%.1f / %.1f GB", s.diskUsedKB / 1048576, s.diskTotalKB / 1048576))
            }
        }
    }

    private func wifi(_ s: PiSample) -> some View {
        GroupBox("Wi-Fi") {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    metric("Señal", s.wifiSignal.map { String(format: "%.0f dBm", $0) } ?? "—",
                           color: (s.wifiSignal ?? 0) < -75 ? .red : (s.wifiSignal ?? 0) < -67 ? .orange : .primary)
                    metric("Calidad", s.wifiLinkQuality.map { String(format: "%.0f/70", $0) } ?? "—")
                    metric("Ping Mac→Pi", m.pingMs.map { String(format: "%.1f ms", $0) } ?? "perdido",
                           color: m.pingMs == nil ? .red : (m.pingMs! > 100 ? .orange : .primary))
                }
                Text([s.ssid.map { "Red \($0)" }, s.wifiIface, s.wifiFreq.map { "\($0) MHz" },
                      s.rxBitrate.map { "rx \($0)" }, s.txBitrate.map { "tx \($0)" }]
                    .compactMap { $0 }.joined(separator: " · "))
                    .font(.caption).foregroundStyle(.secondary)
                Text("IP: " + s.ips.joined(separator: ", ")).font(.caption).foregroundStyle(.secondary)
            }.frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    private func rosSection(_ s: PiSample) -> some View {
        GroupBox("ROS") {
            VStack(alignment: .leading, spacing: 4) {
                if s.ros.isEmpty {
                    Text("No hay nodos ROS en marcha").font(.caption).foregroundStyle(.secondary)
                } else {
                    ForEach(s.ros, id: \.self) { p in
                        HStack {
                            Text(p.name).font(.caption.monospaced())
                            Spacer()
                            Text(Monitor.duration(Double(p.seconds))).font(.caption).foregroundStyle(.secondary)
                        }
                    }
                    if let r = m.ros {
                        Divider()
                        HStack {
                            ForEach(["/scan", "/robotek/imu", "/rrc/battery"], id: \.self) { t in
                                let rate = r.rates[t] ?? nil
                                metric(t, rate.map { String(format: "%.1f Hz", $0) } ?? "no existe",
                                       color: rate == nil || rate == 0 ? .orange : .primary)
                            }
                        }
                        Text("\(r.nodes.count) nodos, \(r.topics) tópicos: " + r.nodes.joined(separator: " "))
                            .font(.caption2).foregroundStyle(.secondary).lineLimit(4)
                    } else if let e = m.rosError {
                        Text(e).font(.caption2).foregroundStyle(.orange).lineLimit(3)
                    } else {
                        Text("Midiendo tópicos…").font(.caption2).foregroundStyle(.secondary)
                    }
                }
            }.frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    // MARK: history

    private var recent: [HistoryPoint] {
        let since = Date().addingTimeInterval(-30 * 60)
        return m.history.filter { $0.date >= since }
    }

    private var charts: some View {
        GroupBox("Últimos 30 min") {
            VStack(spacing: 6) {
                chart("Temperatura °C", recent.compactMap { p in p.tempC.map { (p.date, $0) } }, .red)
                chart("CPU %", recent.compactMap { p in p.cpu.map { (p.date, $0 * 100) } }, .blue)
                chart("Ping ms", recent.compactMap { p in p.pingMs.map { (p.date, $0) } }, .purple)
                chart("Señal dBm", recent.compactMap { p in p.signal.map { (p.date, $0) } }, .teal)
                chart("Batería V", recent.compactMap { p in p.batteryV.map { (p.date, $0) } }, .green)
                chart("5 V del Pi", recent.compactMap { p in p.ext5V.map { (p.date, $0) } }, .orange)
            }
        }
    }

    private func chart(_ title: String, _ points: [(Date, Double)], _ color: Color) -> some View {
        HStack {
            VStack(alignment: .leading) {
                Text(title).font(.caption2).foregroundStyle(.secondary)
                Text(points.last.map { String(format: "%.1f", $0.1) } ?? "—").font(.caption.monospacedDigit())
            }.frame(width: 90, alignment: .leading)
            let values = points.map(\.1)
            Chart(points, id: \.0) { p in
                LineMark(x: .value("t", p.0), y: .value(title, p.1)).foregroundStyle(color)
            }
            .chartXAxis(.hidden)
            .chartYScale(domain: .automatic(includesZero: false))
            .chartYAxis(.hidden)
            .frame(height: 30)
            Text(values.isEmpty ? "" : String(format: "%.1f–%.1f", values.min()!, values.max()!))
                .font(.caption2.monospacedDigit()).foregroundStyle(.secondary).frame(width: 70, alignment: .trailing)
        }
    }

    /// Online (green) / offline (red) strip for the last 2 h.
    private var timeline: some View {
        GroupBox("Conexión, últimas 2 h") {
            GeometryReader { geo in
                let end = Date(), start = end.addingTimeInterval(-7200)
                Canvas { ctx, size in
                    ctx.fill(Path(CGRect(origin: .zero, size: size)), with: .color(.gray.opacity(0.15)))
                    let pts = m.history
                    for (i, p) in pts.enumerated() where p.date >= start {
                        let next = i + 1 < pts.count ? pts[i + 1].date : end
                        let x0 = size.width * p.date.timeIntervalSince(start) / 7200
                        let x1 = size.width * next.timeIntervalSince(start) / 7200
                        ctx.fill(Path(CGRect(x: x0, y: 0, width: max(1, x1 - x0), height: size.height)),
                                 with: .color(p.online ? .green : .red))
                    }
                }
                .frame(width: geo.size.width, height: 14)
            }.frame(height: 14)
            HStack {
                Text("−2 h"); Spacer(); Text("−1 h"); Spacer(); Text("ahora")
            }.font(.caption2).foregroundStyle(.secondary)
        }
    }

    private var eventList: some View {
        GroupBox("Eventos") {
            VStack(alignment: .leading, spacing: 3) {
                if m.events.isEmpty { Text("Sin eventos").font(.caption).foregroundStyle(.secondary) }
                ForEach(m.events.suffix(40).reversed()) { e in
                    HStack(alignment: .top) {
                        Image(systemName: icon(e.kind)).foregroundStyle(eventColor(e.kind)).frame(width: 16)
                        Text(e.date.formatted(Calendar.current.isDateInToday(e.date)
                                              ? .dateTime.hour().minute().second()
                                              : .dateTime.day().month().hour().minute()))
                            .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                        Text(e.text).font(.caption).fixedSize(horizontal: false, vertical: true)
                    }
                }
            }.frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    private var footer: some View {
        HStack {
            Button("Buscar el Pi") { m.scanNow() }
            Button("Abrir ventana") { openWindow(id: "panel"); NSApplication.shared.activate() }
            Button("Abrir registros") { NSWorkspace.shared.open(m.logFolder) }
            Spacer()
            Button("Salir") { NSApplication.shared.terminate(nil) }
        }
    }

    // MARK: helpers

    private func metric(_ title: String, _ value: String, color: Color = .primary) -> some View {
        VStack(alignment: .leading, spacing: 0) {
            Text(title).font(.caption2).foregroundStyle(.secondary).lineLimit(1)
            Text(value).font(.callout.monospacedDigit()).foregroundStyle(color)
        }.frame(maxWidth: .infinity, alignment: .leading)
    }

    private func bar(_ title: String, _ fraction: Double, _ detail: String) -> some View {
        HStack {
            Text(title).font(.caption).frame(width: 50, alignment: .leading)
            ProgressView(value: min(1, max(0, fraction))).tint(fraction > 0.9 ? .red : fraction > 0.8 ? .orange : .accentColor)
            Text(detail).font(.caption.monospacedDigit()).frame(width: 110, alignment: .trailing)
        }
    }

    private func icon(_ kind: String) -> String {
        switch kind {
        case "offline": return "wifi.slash"
        case "online": return "wifi"
        case "reboot": return "arrow.clockwise.circle.fill"
        case "throttle": return "bolt.trianglebadge.exclamationmark"
        case "battery": return "battery.25"
        case "ip": return "network"
        case "ros": return "gearshape.2"
        default: return "info.circle"
        }
    }

    private func eventColor(_ kind: String) -> Color {
        switch kind {
        case "offline", "reboot", "throttle", "battery": return .red
        case "online": return .green
        case "wifi": return .orange
        default: return .secondary
        }
    }
}
