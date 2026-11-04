#!/bin/sh
# Builds PiMonitor.app (menu bar app that shows the health of the car's Raspberry Pi).
#   tools/pi-monitor/build-app.sh          -> tools/pi-monitor/build/PiMonitor.app
#   open tools/pi-monitor/build/PiMonitor.app
set -e
cd "$(dirname "$0")"
swift build -c release
APP=build/PiMonitor.app
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"
cp .build/release/PiMonitor "$APP/Contents/MacOS/PiMonitor"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleIdentifier</key><string>org.robotek.PiMonitor</string>
  <key>CFBundleName</key><string>PiMonitor</string>
  <key>CFBundleExecutable</key><string>PiMonitor</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>LSMinimumSystemVersion</key><string>14.0</string>
  <key>LSUIElement</key><true/>
  <key>NSLocalNetworkUsageDescription</key><string>Busca el Raspberry Pi del coche en la red local (puerto 22).</string>
</dict></plist>
PLIST
codesign --force --sign - "$APP" >/dev/null 2>&1 || true
echo "$APP"
