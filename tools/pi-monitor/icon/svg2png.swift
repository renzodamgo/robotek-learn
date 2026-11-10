// Renders an SVG to a PNG of SIZE x SIZE with transparency (AppKit reads SVG on macOS 14+).
//   swift icon/svg2png.swift in.svg out.png SIZE
import AppKit
let a = CommandLine.arguments
guard let img = NSImage(contentsOfFile: a[1]) else { fatalError("no svg") }
let s = Int(a[3])!
let rep = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: s, pixelsHigh: s, bitsPerSample: 8, samplesPerPixel: 4,
                           hasAlpha: true, isPlanar: false, colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0)!
NSGraphicsContext.saveGraphicsState()
NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: rep)
img.draw(in: NSRect(x: 0, y: 0, width: s, height: s))
NSGraphicsContext.restoreGraphicsState()
try! rep.representation(using: .png, properties: [:])!.write(to: URL(fileURLWithPath: a[2]))
