#!/bin/bash

PACKAGE_NAME="RTI-RoR_X2.pkg"
BUNDLE_NAME="org.rti-zone.RTI-RoRX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libRTI-RoR.dylib
fi

mkdir -p ROOT/tmp/RTI-RoR_X2/
cp "../RTI-RoR.ui" ROOT/tmp/RTI-RoR_X2/
cp "../RTI.png" ROOT/tmp/RTI-RoR_X2/
cp "../domelist RTI-RoR.txt" ROOT/tmp/RTI-RoR_X2/
cp "../build/Release/libRTI-RoR.dylib" ROOT/tmp/RTI-RoR_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
