#!/bin/sh
# Builds "Raspberry Pi Bar.app" (menu bar app that shows the health of the car's Raspberry Pi), with the icon of
# icon/raspberry-pi-bar.svg.
#   tools/pi-monitor/build-app.sh            -> tools/pi-monitor/build/Raspberry Pi Bar.app
#   tools/pi-monitor/build-app.sh --install  -> also copied to /Applications
#   open "/Applications/Raspberry Pi Bar.app"
set -e
cd "$(dirname "$0")"
swift build -c release
NAME="Raspberry Pi Bar"
APP="build/$NAME.app"
rm -rf "$APP" build/PiMonitor.app build/icon.iconset
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources" build/icon.iconset
cp .build/release/PiMonitor "$APP/Contents/MacOS/PiMonitor"
for s in 16 32 128 256 512; do
    swift icon/svg2png.swift icon/raspberry-pi-bar.svg "build/icon.iconset/icon_${s}x${s}.png" "$s"
    swift icon/svg2png.swift icon/raspberry-pi-bar.svg "build/icon.iconset/icon_${s}x${s}@2x.png" "$((s*2))"
done
iconutil -c icns build/icon.iconset -o "$APP/Contents/Resources/AppIcon.icns"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleIdentifier</key><string>org.robotek.PiMonitor</string>
  <key>CFBundleName</key><string>$NAME</string>
  <key>CFBundleDisplayName</key><string>$NAME</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundleExecutable</key><string>PiMonitor</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>LSMinimumSystemVersion</key><string>14.0</string>
  <key>LSUIElement</key><true/>
  <key>NSLocalNetworkUsageDescription</key><string>Busca el Raspberry Pi del coche en la red local (puerto 22).</string>
</dict></plist>
PLIST
codesign --force --sign - "$APP" >/dev/null 2>&1 || true
if [ "$1" = "--install" ]; then
    rm -rf "/Applications/$NAME.app"
    cp -R "$APP" /Applications/
    APP="/Applications/$NAME.app"
fi
echo "$APP"
