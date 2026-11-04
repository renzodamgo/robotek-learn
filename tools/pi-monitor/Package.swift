// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "PiMonitor",
    platforms: [.macOS(.v14)],
    targets: [
        .executableTarget(name: "PiMonitor", path: "Sources/PiMonitor")
    ]
)
