#!/bin/bash
set -e

echo "📦 Packaging Honeycomb for Distribution..."

# Create distribution directory
rm -rf dist
mkdir -p dist/Honeycomb.app/Contents/{MacOS,Resources}

# Copy executable
cp build_production/Honeycomb dist/Honeycomb.app/Contents/MacOS/

# Copy only necessary assets
cp -r assets dist/Honeycomb.app/Contents/Resources/

# Create Info.plist
cat > dist/Honeycomb.app/Contents/Info.plist << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>Honeycomb</string>
    <key>CFBundleIdentifier</key>
    <string>com.yourcompany.honeycomb</string>
    <key>CFBundleName</key>
    <string>Honeycomb</string>
    <key>CFBundleVersion</key>
    <string>1.0</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.15</string>
</dict>
</plist>
EOF

echo "✅ Package created: dist/Honeycomb.app"
du -sh dist/Honeycomb.app